
use crate::local_commands::{LocalCommands};
use crate::protocol::{CanDriveMessage, CommandTypes, Commands, DeviceId, MotorBits, ProtocolData, StatusBits};
use crate::util::{Shared, Threaded, critical_error, get_can_tx_queue, is_running};
use std::collections::btree_map::VacantEntry;
use std::collections::{BTreeMap, BTreeSet};
use std::default;
use std::process::Command;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};
use backtrace::trace;
use crossbeam_channel::{Receiver, Sender, TryRecvError};
use log::{error, info, trace };

/*
Architecture of the device manager

The device manager is responsible for managing the devices that are connected to the CAN bus.
It maintains a database of devices, and provides a mechanism for updating the database based on
messages received from the CAN bus.

The device manager is also responsible for handling discovery requests from the CAN bus. When a
discovery request is received, the device manager will assign a unique ID to the device and send
a discovery assignment message to the CAN bus. The device manager will then wait for a discovery
response message from the device, which will contain the assigned ID.

The device manager is also responsible for handling control messages from the CAN bus. When a
control message is received, the device manager will update the device database based on the
received message.

The device manager is also responsible for handling a subset of local commands. These commands
are used to query the device database and to list the devices that are currently connected to
the CAN bus.

The device manager creates listeners for transactions that require responses, such as discovery requests.
These listeners are stored in a list, and are checked against incoming messages to see if they match
the criteria for the listener. If a match is found, the listener's action is executed.
 */

const DISCOVERY_TIMEOUT_SECS: u64 = 300;

struct IdPool {
    free_ids: BTreeSet<DeviceId>,
}

impl IdPool {
    pub fn new() -> IdPool {
        let btree_values: [DeviceId; 256] = std::array::from_fn::<DeviceId, 256, _>(|f| DeviceId::from_u8(f as u8));
        return IdPool {
            free_ids: BTreeSet::<DeviceId>::from(btree_values),
        };
    }

    pub fn take_next(&mut self, hint: Option<DeviceId>) -> Result<DeviceId, String> {
        let free_ids = &mut self.free_ids;
        let id: DeviceId;
        if let Some(hint) = hint
            && free_ids.contains(&hint)
        {
            id = hint;
        } else if let Some(result) = free_ids.first() {
            id = *result;
        } else {
            return Err("ID is in use".to_string());
        }
        free_ids.remove(&id);
        Ok(id)
    }

    pub fn put(&mut self, id: DeviceId)
    {
        self.free_ids.insert(id);
    }
}

#[derive(Debug, Default, Clone, Copy)]
struct DeviceParameter<T>
where
    T: Copy + PartialEq + std::fmt::Debug,
{
    reported_value: T,
    target_value: T,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Device {
    serial: u32,
    id: DeviceId,
    fw_major: u8,
    fw_minor: u8,
    fw_patch: u8,
    position: DeviceParameter<u16>,
    analog0: DeviceParameter<u16>,
    analog1: DeviceParameter<u16>,
    servo0: DeviceParameter<u16>,
    servo1: DeviceParameter<u16>,
    led_stat: DeviceParameter<u8>,
    led_sys: DeviceParameter<u8>,
    status: StatusBits,
    motor: MotorBits,
    foc1: DeviceParameter<u8>,
    foc2: DeviceParameter<u8>,
    foc3: DeviceParameter<u8>,
    foc_en: DeviceParameter<u8>,
}

pub struct DeviceDatabase {
    filter_actions: Vec<Shared<MessageFilter>>,
    devices: BTreeMap<DeviceId, Device>,
    pending_assignments: BTreeSet<DeviceId>,
    can_tx: Sender<CanDriveMessage>,
    id_pool: IdPool,
}

impl DeviceDatabase {
    pub fn new(can_tx: &Sender<CanDriveMessage>) -> DeviceDatabase {
        DeviceDatabase {
            filter_actions: Vec::new(),
            devices: BTreeMap::new(),
            pending_assignments: BTreeSet::new(),
            can_tx: can_tx.clone(),
            id_pool: IdPool::new(),
        }
    }

    pub fn get_device(&self, id: DeviceId) -> Option<&Device> {
        return self.devices.get(&id);
    }

    pub fn get_device_mut(&mut self, id: &DeviceId) -> Option<&mut Device> {
        return self.devices.get_mut(id);
    }

    fn add_filter(&mut self, filter: MessageFilter) {
        self.filter_actions.push(Shared::new(filter));
    }

    // Provisional assignment of device ID during discovery process
    fn provisional_assign(&mut self, previous_id: &DeviceId, serial: &u32) -> Result<(), String> {
        // Obtain next available ID, prioritizing hinted ID from discovery message
        let previous_id = *previous_id;
        let serial = *serial;
        match self.id_pool.take_next(Some(previous_id)) {
            Ok(assigned_id) => {
                let drop_id = assigned_id;
                self.pending_assignments.insert(assigned_id);
                self.filter_actions.push(Shared::new(MessageFilter::new(
                    Some(assigned_id),
                    Some(CommandTypes::GetInfo),
                    Some((Instant::now(), Duration::from_secs(DISCOVERY_TIMEOUT_SECS), Box::new(move |id_pool: &mut IdPool| { // ON TIMEOUT
                        // Remove assignment from database
                        id_pool.put(drop_id);
                        Err(format!("Device Manager: Expected GetInfo message from dev {} within 5 seconds", assigned_id))
                    }))),
                    Box::new(move |db: &mut BTreeMap<DeviceId, Device>, msg: &CanDriveMessage| { // ON INFO MESSAGE
                        if let CanDriveMessage::Control { id, is_controller: _, cmd, data } = msg {
                            if let Commands::GetInfo = cmd {
                                if let ProtocolData::Info { serial: dev_serial, fw_major, fw_minor, fw_patch } = data {
                                    // Complete assignment
                                    if (serial == *dev_serial) && (*id == assigned_id) {
                                        DeviceDatabase::assign_device(db, &assigned_id, &serial);
                                        if let Some(device_model) = db.get_mut(&assigned_id) {
                                            device_model.fw_major = *fw_major;
                                            device_model.fw_minor = *fw_minor;
                                            device_model.fw_patch = *fw_patch;
                                            info!("Device 0x{:X} (Serial 0x{:X}) added to network", device_model.id, device_model.serial);
                                            Ok(())
                                        } else {
                                            Err(format!("Error fetching device model for {}", id))
                                        }
                                    } else {
                                        Err("Filter failure: Serial mismatch".to_string())
                                    }
                                } else {
                                    Err("Filter failure: Expected GetInfo Data Format".to_string())
                                }
                            } else {
                                Err("Filter failure: Expected GetInfo message".to_string())
                            }
                        } else {
                            Err("Filter failure: Expected Control message".to_string())
                        }
                    }))));
                self.can_tx.send(CanDriveMessage::DiscoveryAssign {
                    serial: serial,
                    assigned_id: assigned_id.to_u8(),
                }).map_err(|e| format!("{}", e))
            }
            Err(e) => Err(e)
        }
    }

    fn assign_device(devices: &mut BTreeMap<DeviceId, Device>, id: &DeviceId, serial: &u32)
    {
        let device = Device {
            serial: *serial,
            id: *id,
            fw_major: 0,
            fw_minor: 0,
            fw_patch: 0,
            position: DeviceParameter::default(),
            analog0: DeviceParameter::default(),
            analog1: DeviceParameter::default(),
            servo0: DeviceParameter::default(),
            servo1: DeviceParameter::default(),
            led_stat: DeviceParameter::default(),
            led_sys: DeviceParameter::default(),
            status: StatusBits::default(),
            motor: MotorBits::default(),
            foc1: DeviceParameter::default(),
            foc2: DeviceParameter::default(),
            foc3: DeviceParameter::default(),
            foc_en: DeviceParameter::default(),
        };
        devices.insert(*id, device);
    }

    pub fn handle_device_message(&mut self, id: &DeviceId, is_controller: &bool, cmd: &Commands, data: &ProtocolData) -> Result<(), String>
    {
        if *is_controller {
            return Err("Controllers cannot be updated".to_string());
        }
        trace!("Handling device message from {id} ({cmd:?})");
        // Check and prune filters
        let mut msg_handled = false;
        {
            let mut devices = &mut self.devices;
            let mut id_pool = &mut self.id_pool;
            self.filter_actions.retain(|filter| {
                let filter = filter.get();
                if let Some(id_filter) = &filter.id_filter {
                    if *id_filter != *id {
                        return true;
                    }
                }
                if let Some(cmd_type) = &filter.message_filter {
                    if cmd_type != &cmd.into() {
                        return true;
                    }
                }

                if let Some((start, timeout, action)) = &filter.timeout {
                    if start.elapsed() > *timeout
                    {
                        msg_handled = true;
                        match action(&mut id_pool)
                        {
                            Ok(_) => return false,
                            Err(e) => {
                                error!("{}",e);
                                return false;
                            }
                        }
                    }
                }
                // We match the filter, execute the action
                msg_handled = true;
                if let Some(action) = &filter.action {
                    match (action)(&mut devices, &CanDriveMessage::Control {
                        id: *id,
                        is_controller: *is_controller,
                        cmd: *cmd,
                        data: *data,
                    })
                    {
                        Ok(_) => (),
                        Err(e) => {
                            error!("Error executing filter action: {}", e);
                        }
                    }
                }
                false
            });
        }
        // If not handled by a filter action, handle it here
        if !msg_handled {
            trace!("Message not handled by filters, handling directly");
            if let Some(device_model) = self.get_device_mut(&id) {
                match cmd {
                    Commands::GetInfo => {
                        if let ProtocolData::Info {
                            serial: _,
                            fw_major,
                            fw_minor,
                            fw_patch,
                        } = data
                        {
                            // Serial is read only once discovered
                            //device_model.serial = serial;
                            device_model.fw_major = *fw_major;
                            device_model.fw_minor = *fw_minor;
                            device_model.fw_patch = *fw_patch;
                        }
                    }
                    Commands::GetPosition => {
                        if let ProtocolData::Position { value } = data {
                            device_model.position.reported_value = *value;
                        }
                    }
                    Commands::GetStatus => {
                        if let ProtocolData::Status(status) = data {
                            device_model.status = *status;
                        }
                    }
                    Commands::GetAnalog => {
                        if let ProtocolData::Analog { a0, a1 } = data {
                            device_model.analog0.reported_value = *a0;
                            device_model.analog1.reported_value = *a1;
                        }
                    }
                    Commands::GetServo => {
                        if let ProtocolData::Servo { s0, s1 ,update_flag } = data {
                            if *update_flag & 0x01 == 0x01 {
                                device_model.servo0.target_value = *s0;
                            }
                            if *update_flag & 0x02 == 0x02 {
                                device_model.servo1.target_value = *s1;
                            }
                        }
                    }
                    Commands::GetLed => {
                        if let ProtocolData::Led { sys, stat, update_flag } = data {
                            if *update_flag & 0x01 == 0x01 {
                                device_model.led_stat.reported_value = *stat;
                            }
                            if *update_flag & 0x02 == 0x02 {
                                device_model.led_sys.reported_value = *sys;
                            }
                        }
                    }
                    Commands::GetMotor => {
                        if let ProtocolData::Motor(motor) = data {
                            device_model.motor = *motor;
                        }
                    }
                    Commands::GetFOC => {
                        if let ProtocolData::FOC {
                            foc_1,
                            foc_2,
                            foc_3,
                            en,
                        } = data
                        {
                            device_model.foc1.reported_value = *foc_1;
                            device_model.foc2.reported_value = *foc_2;
                            device_model.foc3.reported_value = *foc_3;
                            device_model.foc_en.reported_value = *en;
                        }
                    }
                    _ => {
                        // unimplemented
                        return Err("Warning: Unhandled device command".to_string());
                    }
                }
                // Update complete
                Ok(())
            } else {
                return Err("Error: Undiscovered device".to_string());
            }
        } else {
            // No updates required, nothing to do
            Ok(())
        }
    }

    fn discover_device(&mut self, serial: &u32, previous_id: &DeviceId) -> Result<(), String> {
        trace!("Device Manager: Handling discovery message: Serial: {:X} Previous ID: {:X}", serial, previous_id);
        match self.provisional_assign(previous_id, serial) {
            Ok(_) => Ok(()),
            Err(e) => Err(e)
        }
    }
}

type MessageFilterAction = Box<dyn Fn(&mut BTreeMap<DeviceId, Device>, &CanDriveMessage) -> Result<(), String> + Send>;
type TimeoutAction = Box<dyn Fn(&mut IdPool) -> Result<(), String> + Send>;

struct MessageFilter {
    id_filter: Option<DeviceId>,
    message_filter: Option<CommandTypes>,
    timeout: Option<(Instant, Duration, TimeoutAction)>,
    action: Option<MessageFilterAction>,
}

impl MessageFilter {
    fn new(id_filter: Option<DeviceId>, message_filter: Option<CommandTypes>, timeout: Option<(Instant, Duration, TimeoutAction)>, action: MessageFilterAction) -> Self {
        MessageFilter {
            id_filter: id_filter,
            message_filter: message_filter,
            timeout: timeout,
            action: Some(action),
        }
    }

    fn expect_ack(id: DeviceId, cmd: CommandTypes) -> MessageFilter
    {
        let dev_id = id;
        let query_cmd = cmd;
        MessageFilter {
            id_filter: Some(id),
            message_filter: Some(cmd),
            timeout: Some((Instant::now(), Duration::from_secs(5), Box::new(move |id_pool: &mut IdPool| {
                get_can_tx_queue().send(CanDriveMessage::Control {
                    id: dev_id,
                    is_controller: true,
                    cmd: Commands::RevokeConfig,
                    data: ProtocolData::Empty,
                }).map_err(|e| format!("{}", e))?;
                id_pool.put(dev_id);
                Ok(())
            }))),
            action: Some(Box::new(move |_, _| {
                info!("Device {dev_id}: Acknowledged {query_cmd:?}");
                Ok(())
            })),
        }
    }
}

pub struct DeviceManager {
    //database: Shared<DeviceDatabase>,
    can_tx: Sender<CanDriveMessage>,
    thread: Option<JoinHandle<()>>,
}

impl DeviceManager {
    pub fn new(
        can_tx: &Sender<CanDriveMessage>,
        can_rx: &Receiver<CanDriveMessage>,
        local_rx: &Receiver<LocalCommands>,
    ) -> DeviceManager {
        let db = Shared::new(DeviceDatabase {
            filter_actions: Vec::new(),
            devices: BTreeMap::new(),
            pending_assignments: BTreeSet::new(),
            can_tx: can_tx.clone(),
            id_pool: IdPool::new(),
        });
        let inner_local_rx = local_rx.clone();
        let inner_can_rx = can_rx.clone();
        let db_handle = db.clone();
        DeviceManager {
            //database: db.clone(),
            can_tx: can_tx.clone(),
            thread: Some(std::thread::spawn(move || {
                trace!("Entered device manager thread");
                while is_running() {
                    match inner_local_rx.try_recv() {
                        Ok(msg) => {
                            trace!("\nDequeing local command: {:?}", msg);
                            match
                            DeviceManager::handle_local_command(db_handle.clone(), msg)
                            {
                                Ok(_) => (),
                                Err(e) => {
                                    critical_error(format!("Error handling local command: {}", e));
                                    break;
                                }
                            }
                        }
                        Err(TryRecvError::Empty) => {},
                        Err(e) => {
                            critical_error(format!("Error receiving message: {}", e));
                            break;
                        }
                    }
                    match inner_can_rx.try_recv() {
                        Ok(msg) => {
                            trace!("Dequeing protocol message: {:?}", msg);
                            match &msg {
                                CanDriveMessage::DiscoveryReq {
                                    serial,
                                    previous_id,
                                } => {
                                    match db_handle.get().discover_device(&serial, &DeviceId::from_u8(*previous_id)) {
                                        Ok(_) => (),
                                        Err(e) => {
                                            error!("Error handling discovery message: {}", e);
                                            break;
                                        }
                                    }
                                },
                                CanDriveMessage::DiscoveryAssign {
                                    serial: _,
                                    assigned_id: _,
                                } => {
                                    // TODO disable authoritative mode
                                    error!("Warning: Second controller detected, conflicts likely")
                                },
                                CanDriveMessage::Control { id, is_controller,cmd, data } => {
                                    if *is_controller {
                                        trace!("Logging Controller message: ID{:?} [{:?}]", id, data);
                                    } else {
                                        match db_handle.get().handle_device_message(id, is_controller, cmd, data) {
                                            Ok(_) => (),
                                            Err(e) => {
                                                error!("Error handling control message: {}", e);
                                                break;
                                            }
                                        }
                                    }
                                },
                                CanDriveMessage::Broadcast { cmd, data } => {
                                    trace!("Logging Broadcast message: {:?} {:?}", cmd, data);
                                }
                            }
                        }
                        Err(TryRecvError::Empty) => {},
                        Err(e) => {
                            critical_error(format!("Error receiving message: {}", e));
                            break;
                        }
                    }
                }
            }))
        }
    }

    pub fn reset_network(&mut self) -> Result<(), String> {
        self.broadcast(CanDriveMessage::Broadcast { cmd: Commands::NetworkReset, data: ProtocolData::Empty })
    }

    fn handle_local_command(db: Shared<DeviceDatabase>, msg: LocalCommands) -> Result<(), String> {
        //let can_tx = get_can_tx_queue();
        let mut db = db.get();
        match msg {
            LocalCommands::ListDevices => {
                db.devices.iter().for_each(|(id, dev)| {
                    info!("Device ID: 0x{:X} Serial: 0x{:X}", id, dev.serial);
                });
                Ok(())
            },
            LocalCommands::SetPosition(id, pos) => {
                DeviceManager::device_query(&mut db, id, Commands::SetPosition, ProtocolData::Position{ value: pos})
            },
            LocalCommands::GetPosition(id) => {
                DeviceManager::device_query(&mut db, id, Commands::GetPosition, ProtocolData::Empty)
            },
            LocalCommands::GetInfo(id) => {
                DeviceManager::device_query(&mut db, id, Commands::GetInfo, ProtocolData::Empty)
            },
            LocalCommands::GetStatus(id) => {
                DeviceManager::device_query(&mut db, id, Commands::GetStatus, ProtocolData::Empty)
            },
            LocalCommands::SetSysLed(id, value) => {
                DeviceManager::device_query(&mut db, id, Commands::SetLed, ProtocolData::Led {
                    sys: value,
                    stat: 0,
                    update_flag: 0x01
                })
            },
            LocalCommands::SetStatLed(id, value) => {
                DeviceManager::device_query(&mut db, id, Commands::SetLed, ProtocolData::Led {
                    sys: 0,
                    stat: value,
                    update_flag: 0x02
                })
            }
            LocalCommands::GetLeds(id) => {
                DeviceManager::device_query(&mut db, id, Commands::GetLed, ProtocolData::Empty)
            },
            LocalCommands::GetMotor(id) => {
                DeviceManager::device_query(&mut db, id, Commands::GetMotor, ProtocolData::Empty)
            },
            LocalCommands::GetAnalog(id) => {
                DeviceManager::device_query(&mut db, id, Commands::GetAnalog, ProtocolData::Empty)
            },
            LocalCommands::GetServo(id) => {
                DeviceManager::device_query(&mut db, id, Commands::GetServo, ProtocolData::Empty)
            },
            LocalCommands::SetServo0(id, pos) => {
                DeviceManager::device_query(&mut db, id, Commands::SetServo, ProtocolData::Servo { s0: pos, s1: 0, update_flag: 0x01 })
            },
            LocalCommands::SetServo1(id, pos) => {
               DeviceManager::device_query(&mut db, id, Commands::SetServo, ProtocolData::Servo { s0: 0, s1: pos, update_flag: 0x02 })
            },
            LocalCommands::UpdateFirmware(_id, _path) => {
                Err("Update not implemented yet".to_string())
            },
            LocalCommands::RevokeConfig(id) => {
                DeviceManager::device_query(&mut db, id, Commands::RevokeConfig, ProtocolData::Empty)
            },
            _ => Ok(())
        }
    }

    pub fn device_query(
        db: &mut DeviceDatabase,
        query_id: DeviceId,
        query_cmd: Commands,
        query_data: ProtocolData,
    ) -> Result<(), String> {
        trace!("Starting device query of {query_id} ({query_cmd:?} {query_data:?}");
        const DEFAULT_QUERY_TIMEOUT: u64 = 30;
        let query_id = query_id.clone();
        let query_cmd = query_cmd.clone();
        let can_tx = get_can_tx_queue();
        let ta: TimeoutAction = Box::new(move |id_pool: &mut IdPool| -> Result<(), String> {
            error!("Device {query_id} {query_cmd:?}: Timeout");
            trace!("{query_data:?}");
            get_can_tx_queue().send(CanDriveMessage::Control {
                id: query_id,
                is_controller: true,
                cmd: Commands::RevokeConfig,
                data: ProtocolData::Empty,
            }).map_err(|e| format!("{}", e))?;
            id_pool.put(query_id);
            Ok(())
        });
        let default_timeout = (Instant::now(),Duration::from_secs(DEFAULT_QUERY_TIMEOUT), ta);
        can_tx.send(CanDriveMessage::Control { id: query_id, is_controller: true, cmd: query_cmd, data: query_data }).map_err(|e| format!("{}", e))?;
        match query_cmd.into() {
            CommandTypes::GetInfo => {
                db.add_filter(MessageFilter::new(Some(query_id), Some(CommandTypes::GetInfo), Some(default_timeout), Box::new(move | _db, msg|
                {
                    if let CanDriveMessage::Control { id: _, is_controller:_, cmd: _, data } = msg {
                        if let ProtocolData::Info { serial, fw_major, fw_minor, fw_patch } = data
                        {
                            info!("Device {query_id} {query_cmd:?}: {serial:X} {fw_major}.{fw_minor}.{fw_patch}");
                            return Ok(());
                        }
                    }
                    Err("Filter failure: Bad message type".to_string())
                })));
                Ok(())
            },
            CommandTypes::GetInfoExt => {
                db.add_filter(MessageFilter::new(Some(query_id), Some(CommandTypes::GetInfoExt), Some(default_timeout), Box::new(move | _db, msg|
                {
                    if let CanDriveMessage::Control { id: _, is_controller:_, cmd: _, data } = msg {
                        if let ProtocolData::InfoExt { flags, temperature } = data
                        {
                            info!("Device {query_id} {query_cmd:?}: Flags: {flags} Temperature: {temperature}");
                            return Ok(());
                        }
                    }
                    Err("Filter failure: Bad message type".to_string())
                })));
                Ok(())
            },
            CommandTypes::GetPosition => {
                db.add_filter(MessageFilter::new(Some(query_id), Some(CommandTypes::GetPosition), Some(default_timeout), Box::new(move | _db, msg|
                {
                    if let CanDriveMessage::Control { id: _, is_controller:_, cmd: _, data } = msg {
                        if let ProtocolData::Position { value } = data
                        {
                            info!("Device {query_id} {query_cmd:?}: Position: {value}");
                            return Ok(());
                        }
                    }
                    Err("Filter failure: Bad message type".to_string())
                })));
                Ok(())
            },
            CommandTypes::GetStatus => {
                db.add_filter(MessageFilter::new(Some(query_id), Some(CommandTypes::GetStatus), Some(default_timeout), Box::new(move | _db, msg|
                {
                    if let CanDriveMessage::Control { id: _, is_controller:_, cmd: _, data } = msg {
                        if let ProtocolData::Status(status) = data
                        {
                            let endstop0 = status.endstop0;
                            let endstop1 = status.endstop1;
                            let misc = status.misc;
                            let fault = status.fault;
                            let mag = status.mag;
                            info!("Device {query_id} {query_cmd:?}: E0: {endstop0} E1: {endstop1} M: {misc} F: {fault} M: {mag}");
                            return Ok(());
                        }
                    }
                    Err("Filter failure: Bad message type".to_string())
                })));
                Ok(())
            },
            CommandTypes::GetAnalog => {
                db.add_filter(MessageFilter::new(Some(query_id), Some(CommandTypes::GetAnalog), Some(default_timeout), Box::new(move | _db, msg|
                {
                    if let CanDriveMessage::Control { id: _, is_controller:_, cmd: _, data } = msg {
                        if let ProtocolData::Analog {a0, a1} = data
                        {
                            info!("Device {query_id} {query_cmd:?}: A0: {a0} A1: {a1}");
                            return Ok(());
                        }
                    }
                    Err("Filter failure: Bad message type".to_string())
                })));
                Ok(())
            },
            CommandTypes::GetServo => {
                db.add_filter(MessageFilter::new(Some(query_id), Some(CommandTypes::GetServo), Some(default_timeout), Box::new(move | _db, msg|
                {
                    if let CanDriveMessage::Control { id: _, is_controller:_, cmd: _, data } = msg {
                        if let ProtocolData::Servo { s0, s1, update_flag } = data
                        {
                            info!("Device {query_id} {query_cmd:?}: S0: {s0} S1: {s1} Update Flag: {update_flag}");
                            return Ok(());
                        }
                    }
                    Err("Filter failure: Bad message type".to_string())
                })));
                Ok(())
            },
            CommandTypes::GetMotor => {
                db.add_filter(MessageFilter::new(Some(query_id), Some(CommandTypes::GetMotor), Some(default_timeout), Box::new(move | _db, msg|
                {
                    if let CanDriveMessage::Control { id: _, is_controller:_, cmd: _, data } = msg {
                        if let ProtocolData::Motor(motor) = data
                        {
                            let value = motor.value;
                            let rst = motor.rst;
                            let sleep = motor.sleep;
                            info!("Device {query_id} {query_cmd:?}: M: {value} RST: {rst} Sleep: {sleep}");
                            return Ok(());
                        }
                    }
                    Err("Filter failure: Bad message type".to_string())
                })));
                Ok(())
            },
            CommandTypes::GetLed => {
                db.add_filter(MessageFilter::new(Some(query_id), Some(CommandTypes::GetLed), Some(default_timeout), Box::new(move | _db, msg|
                {
                    if let CanDriveMessage::Control { id: _, is_controller:_, cmd: _, data } = msg {
                        if let ProtocolData::Led { sys, stat, update_flag } = data
                        {
                            info!("Device {query_id} {query_cmd:?}: Sys: {sys} Stat: {stat} Update Flag: {update_flag}");
                            return Ok(());
                        }
                    }
                    Err("Filter failure: Bad message type".to_string())
                })));
                Ok(())
            },
            CommandTypes::GetFOC => {
                db.add_filter(MessageFilter::new(Some(query_id), Some(CommandTypes::GetFOC), Some(default_timeout), Box::new(move | _db, msg|
                    {
                        if let CanDriveMessage::Control { id: _, is_controller:_, cmd: _, data } = msg {
                            if let ProtocolData::FOC {
                                foc_1,
                                foc_2,
                                foc_3,
                                en,
                            } = data
                            {
                                info!("Device {query_id} {query_cmd:?}: Foc1: {foc_1} Foc2: {foc_2} Foc3: {foc_3} En: {en}");
                                return Ok(());
                            }
                        }
                        Err("Filter failure: Bad message type".to_string())
                    })));
                Ok(())
            },
            CommandTypes::RevokeConfig => {
                Ok(())
            },
            CommandTypes::Error => {
                Err("Error is not queryable".to_string())
            },
            CommandTypes::NetworkReset => {
                Err("NetworkReset is not queryable".to_string())
            },
            CommandTypes::Invalid => {
                Err("Invalid command".to_string())
            },
            _ => {
                db.add_filter(MessageFilter::expect_ack(query_id, CommandTypes::Ack));
                Ok(())
            }
        }
    }

    pub fn broadcast(&self, msg: CanDriveMessage) -> Result<(), String> {
        // Broadcast does not expect a response
        self.can_tx.send(msg).map_err(|e| format!("{}", e))
    }
}

impl Threaded for DeviceManager {
    fn join(&mut self) {
        self.thread.take().unwrap().join().unwrap();
    }
}