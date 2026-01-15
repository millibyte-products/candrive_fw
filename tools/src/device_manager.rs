
use crate::local_commands::{LocalCommands};
use crate::protocol::{CanDriveMessage, Commands, MotorBits, ProtocolData, StatusBits, DeviceId};
use crate::util::Threaded;
use std::borrow::BorrowMut;
use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, LazyLock, Mutex};
use std::thread::JoinHandle;
use crossbeam_channel::{Receiver, Sender, TryRecvError, unbounded};
use std::time::Duration;

pub struct IdPool {
    free_ids: BTreeSet<u8>,
}

impl IdPool {
    pub fn new() -> IdPool {
        let btree_values: [u8; 256] = std::array::from_fn::<u8, 256, _>(|f| f as u8);
        return IdPool {
            free_ids: BTreeSet::<u8>::from(btree_values),
        };
    }

    pub fn take_next(&mut self, hint: Option<u8>) -> Result<u8, String> {
        let free_ids = &mut self.free_ids;
        let id: u8;
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
struct Device {
    serial: u32,
    fw_version: u8,
    hw_version: u8,
    assigned_id: u8,
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

struct DeviceManager {
    devices: BTreeMap<DeviceId, Device>,
    id_pool: IdPool,
    can_message_queue_tx: Sender<CanDriveMessage>,
    can_message_queue_rx: Receiver<CanDriveMessage>,
    thread: Option<JoinHandle<()>>,
}

impl DeviceManager {
    pub fn new(
        can_tx: &Sender<CanDriveMessage>,
        can_rx: &Receiver<CanDriveMessage>,
        local_rx: &Receiver<LocalCommands>,
    ) -> Arc<Mutex<DeviceManager>> {
        let devm = Arc::new(Mutex::new(DeviceManager {
            devices: BTreeMap::new(),
            id_pool: IdPool::new(),
            can_message_queue_tx: can_tx.clone(),
            can_message_queue_rx: can_rx.clone(),
            thread: None
        }));
        let devm_handle = devm.clone();
        let local_msg_queue = local_rx.clone();
        let can_msg_queue = can_rx.clone();
        let thread = std::thread::spawn(move || {
           loop {
               match local_msg_queue.try_recv() {
            Ok(msg) => {
                println!("Received message: {:?}", msg);
                let mut devm_locked = devm_handle.lock().unwrap();
                match devm_locked.handle_command(msg) {
                    Ok(_) => {},
                    Err(e) => {
                        eprintln!("Error handling message: {}", e);
                    }
                }
            }
            Err(TryRecvError::Empty) => {},
            Err(e) => { eprintln!("Error receiving message: {}", e); }
        }
        match can_msg_queue.try_recv() {
            Ok(msg) => {
                println!("Received message: {:?}", msg);
                let mut devm_locked = devm_handle.lock().unwrap();
                match devm_locked.handle_device_message(&msg) {
                    Ok(_) => {},
                    Err(e) => {
                        eprintln!("Error handling message: {}", e);
                    }
                }
            }
            Err(TryRecvError::Empty) => {},
            Err(e) => {
                eprintln!("Error receiving message: {}", e);
            }
        }
            }
        });
        devm.lock().unwrap().borrow_mut().thread = Some(thread);
        return devm;
    }

    pub fn get_device(&self, id: DeviceId) -> Option<&Device> {
        return self.devices.get(&id);
    }

    pub fn get_device_mut(&mut self, id: &DeviceId) -> Option<&mut Device> {
        return self.devices.get_mut(id);
    }

    pub fn assign_device(&mut self, id: &DeviceId, serial: &u32) -> Result<DeviceId, String> {
        match self.id_pool.take_next(Some(id.to_u8())) {
            Ok(assigned_id) => {
                let dev_id = DeviceId::new(assigned_id);
                let device = Device {
                    serial: *serial,
                    fw_version: 0,
                    hw_version: 0,
                    assigned_id: assigned_id,
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
                self.devices.insert(dev_id, device);
                Ok(dev_id)
            }
            Err(e) => Err(e),
        }
    }

    pub fn handle_command(&mut self, msg: LocalCommands) -> Result<(), String> {
        match msg {
            LocalCommands::ListDevices => {
                self.devices.iter().for_each(|(id, dev)| {
                    println!("Device ID: {:?} Serial: {:?}", id, dev.serial);
                });
                Ok(())
            },
            _ => Ok(())
        }
    }

    pub fn handle_device_message(&mut self, msg: &CanDriveMessage) -> Result<(), String> {
        match msg {
            CanDriveMessage::DiscoveryReq {
                serial,
                previous_id,
            } => {
                self.handle_discovery(serial, previous_id)
            },
            CanDriveMessage::DiscoveryAssign {
                serial,
                assigned_id,
            } => {
                // TODO disable authoritative mode
                Err("Warning: Second controller detected, conflicts likely".to_string())
            },
            _ => self.update_device(msg)
        }
    }

    pub fn handle_discovery(&mut self, dev_serial: &u32, previous_id: &u8) -> Result<(), String> {
        let assigned_id: DeviceId;
        match self.assign_device(&DeviceId::new(*previous_id), dev_serial) {
            Ok(id) => {
                println!("Assigned ID: {:?} to serial: {:?}", id, dev_serial);
                assigned_id = id;
            }
            Err(e) => {
                println!("Error assigning ID to serial: {:?}: {:?}", dev_serial, e);
                return Err("Could not assign id to device".to_string())
            }
        };
        // Notify device of assignment
        match self.can_message_queue_tx.send(CanDriveMessage::DiscoveryAssign {
            serial: *dev_serial,
            assigned_id: assigned_id.to_u8(),
        }) {
            Ok(_) => {},
            Err(e) => {
                return Err("Could send assign msg to device".to_string())
            }
        };
        // Wait for device ack
        match self.wait_for_message(|msg: &CanDriveMessage| -> bool {
            match msg {
                CanDriveMessage::Control { id , is_controller, cmd, data } => {
                    // Check for ack integrity
                    if !is_controller &&
                        *id == assigned_id
                        && matches!(cmd, Commands::GET_INFO)
                    {
                        if let ProtocolData::Info {
                            serial,
                            fw_version,
                            hw_version,
                        } = data
                        {
                            if *dev_serial == *serial {
                                return true;
                            }
                        }
                    }
                    false
                }
                _ => false,
            }
        }) {
            Ok(msg) => {
                // Will update device from info struct
                self.handle_device_message(&msg)
            },
            Err(e) => Err("Could not assign device".to_string()), // Don't rewind device assignment, just use the next one
        }
    }

    fn update_device(&mut self, msg: &CanDriveMessage) -> Result<(), String>
    {
        if let CanDriveMessage::Control { id, is_controller, cmd, data } = msg {
            if let Some(device_model) = self.get_device_mut(&id) {
                if *is_controller {
                    return Err("Controllers cannot be updated".to_string());
                }
                match cmd {
                    Commands::GET_INFO => {
                        if let ProtocolData::Info {
                            serial,
                            fw_version,
                            hw_version,
                        } = data
                        {
                            // Read only once
                            //device_model.serial = serial;
                            device_model.fw_version = *fw_version;
                            device_model.hw_version = *hw_version;
                        }
                    }
                    Commands::GET_POSITION => {
                        if let ProtocolData::Position { value } = data {
                            device_model.position.reported_value = *value;
                        }
                    }
                    Commands::GET_STATUS => {
                        if let ProtocolData::Status(status) = data {
                            device_model.status = *status;
                        }
                    }
                    Commands::GET_ANALOG => {
                        if let ProtocolData::Analog { a0, a1 } = data {
                            device_model.analog0.reported_value = *a0;
                            device_model.analog1.reported_value = *a1;
                        }
                    }
                    Commands::GET_SERVO => {
                        if let ProtocolData::Servo { s0, s1 ,update_flag } = data {
                            if *update_flag & 0x01 == 0x01 {
                                device_model.servo0.target_value = *s0;
                            }
                            if *update_flag & 0x02 == 0x02 {
                                device_model.servo1.target_value = *s1;
                            }
                        }
                    }
                    Commands::GET_LED => {
                        if let ProtocolData::Led { sys, stat, update_flag } = data {
                            if *update_flag & 0x01 == 0x01 {
                                device_model.led_stat.reported_value = *stat;
                            }
                            if *update_flag & 0x02 == 0x02 {
                                device_model.led_sys.reported_value = *sys;
                            }
                        }
                    }
                    Commands::GET_MOTOR => {
                        if let ProtocolData::Motor(motor) = data {
                            device_model.motor = *motor;
                        }
                    }
                    Commands::GET_FOC => {
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
                        return Err("Warning: Unhandled device command".to_string())
                    }
                }
                // Update complete
                Ok(())
            } else {
                // No updates required
                Ok(())
            }
        } else {
            Err("Error: Undiscovered device".to_string())
        }

    }

    // Handles messages until a discriminant matches
    pub fn wait_for_message(
        &mut self,
        predicate: impl Fn(&CanDriveMessage) -> bool) -> Result<CanDriveMessage, String>
    {
        loop {
            match self.can_message_queue_rx.try_recv() {
                Ok(msg) => {
                    // Check for ack integrity
                    if predicate(&msg) {
                        return Ok(msg);
                    }
                },
                Err(TryRecvError::Empty) => {},
                Err(e) => { return Err(format!("Error receiving message: {}", e))}
            }
        }
    }

    pub fn transaction(
        &mut self,
        query_id: DeviceId,
        query_cmd: Commands,
        query_data: ProtocolData,
    ) -> Result<(), String> {
        if let Some(mut device_model) = self.get_device_mut(&query_id) {
            match self.can_message_queue_tx.send(CanDriveMessage::Control { id: query_id, is_controller: true, cmd: query_cmd, data: query_data }) {
                Ok(_) => {},
                Err(e) => {
                    return Err("Could not send control msg to device".to_string())
                }
            }
            match self.wait_for_message(|msg| -> bool {
                match msg {
                    CanDriveMessage::Control {
                        id,
                        is_controller,
                        cmd,
                        data,
                    } => {
                        if !*is_controller {
                            // Check for ack integrity
                            if query_id == *id && matches!(query_cmd, cmd) {
                                return true;
                            }
                        }
                        false
                    }
                    _ => false,
                }
            }) {
                Ok(msg) => {
                    // Handle reply
                    self.handle_device_message(&msg)
                }
                Err(e) => Err(e), // Don't rewind device assignment, just use the next one
            }
        } else {
            Err("Device not found".to_string())
        }
    }
}

impl Threaded for DeviceManager {
    fn join(&mut self) {
        self.thread.take().unwrap().join().unwrap();
    }
}