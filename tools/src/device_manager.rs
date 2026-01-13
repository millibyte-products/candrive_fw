use protocol::{CanDriveMessage, Commands, ProtocolData};
use std::mem;
use std::sync::mpsc::{Receiver, Sender, channel};
use std::time::Duration;
use tokio::sync::CancellationToken;

struct IdPool {
    free_ids: BTreeSet<u8>,
}

impl IdPool {
    fn new() -> IdPool {
        return IdPool {
            free_ids: BTreeSet::from([0..255]),
        };
    }

    fn take_next(&mut self, hint: Option<u8>) -> Result<u8, String> {
        let mut free_ids = &mut self.free_ids;
        if let Some(hint) = hint
            && free_ids.contains(&hint)
        {
            free_ids.remove(&hint);
            Ok(hint)
        } else {
            match free_ids.first() {
                Some(id) => {
                    let result = *id;
                    free_ids.remove(id);
                    Ok(result)
                }
                None => Err("No available IDs".to_string()),
            }
        }
    }
}

// Note: static items do not call [`Drop`] on program termination, so this won't be deallocated.
// this is fine, as the OS can deallocate the terminated program faster than we can free memory
// but tools like valgrind might report "memory leaks" as it isn't obvious this is intentional.
static ID_POOL: LazyLock<IdPool> = LazyLock::new(|| IdPool::new());

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
    servo0: DeviceParameter<u8>,
    servo1: DeviceParameter<u8>,
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
    transmitter: Sender<CanDriveMessage>,
    receiver: Receiver<CanDriveMessage>,
    ipc_tx: Sender<LocalCommand>,
    ipc_rx: Receiver<LocalCommand>,
    cancellation_token: CancellationToken,
    timeout: Duration,
}

impl DeviceManager {
    pub fn new(
        can_txrx: IPCChannel<CanDriveMessage>,
        local_txrx: IPCChannel<LocalCommand>,
        cancellation_token: CancellationToken,
        timeout: Option<Duration>,
    ) -> DeviceManager {
        let (ipc_tx, message_queue) = channel::<LocalCommand>();
        let (ext_message_queue, ipc_rx) = channel::<LocalCommand>();
        return DeviceManager {
            devices: BTreeMap::new(),
            id_pool: IdPool::new(),
            transmitter,
            receiver,
            ipc_tx,
            ipc_rx,
            cancellation_token,
            timeout: timeout.unwrap_or(Duration::from_secs(5)),
        };
    }

    pub fn get_channel(&self) -> (Sender<LocalCommand>, Receiver<LocalCommand>) {
        return self.local_txrx.get_channel();
    }

    pub fn get_device(&self, id: DeviceId) -> Option<&Device> {
        return self.devices.get(&id);
    }

    pub fn get_device_mut(&mut self, id: DeviceId) -> Option<&mut Device> {
        return self.devices.get_mut(&id);
    }

    pub fn assign_device(&mut self, id: DeviceId, serial: u32) -> Result<DeviceId, &'static str> {
        match self.id_pool.take_next(Some(id.to_u8())) {
            Ok(assigned_id) => {
                let device = Device {
                    serial,
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
                self.devices.insert(assigned_id, device);
                Ok(assigned_id)
            }
            Err(e) => Err(e),
        }
    }

    pub async fn default_message_handler(&mut self, msg: CanDriveMessage) -> Result<(), String> {
        match msg {
            CanDriveMessage::DiscoveryReq {
                serial,
                previous_id,
            } => {
                match self.assign_device(DeviceId::new(previous_id), serial) {
                    Ok(id) => println!("Assigned ID: {:?} to serial: {:?}", id, serial),
                    Err(e) => {
                        println!("Error assigning ID to serial: {:?}: {:?}", serial, e);
                        return Err(e);
                    }
                }
                self.can_txrx
                    .send(CanDriveMessage::DiscoveryAssign {
                        serial,
                        assigned_id: id,
                    })
                    .await?;

                match self.wait_for_message(|msg: &CanDriveMessage| -> bool {
                    match msg {
                        CanDriveMessage::Control { ack_id, cmd, data } => {
                            // Check for ack integrity
                            if ack_id == id
                                && cmd == Commands::DEVICE_INFO
                            {
                                if let ProtocolData::Info {
                                    ack_serial,
                                    fw_version,
                                    hw_version,
                                } = data
                                {
                                    if ack_serial == serial {
                                        let mut device = self.get_device_mut(id).unwrap();
                                        device.serial = serial;
                                        device.fw_version = fw_version;
                                        device.hw_version = hw_version;
                                        return true;
                                    }
                                }
                            }
                            false
                        }
                        _ => false,
                    }
                }) {
                    Ok(msg) => Ok(()),
                    Err(e) => Err(e), // Don't rewind device assignment, just use the next one
                }
            }
            CanDriveMessage::DiscoveryAssign {
                serial,
                assigned_id,
            } => {
                println!("Warning: Second controller detected, conflicts likely");
            },
            CanDriveMessage::Control { id, cmd, data } => {
                if let Some(device_model) = self.get_device_mut(id) {
                    if cmd.is_device_command() {
                        match cmd {
                            Commands::DEVICE_INFO => {
                                if let ProtocolData::Info {
                                    serial,
                                    fw_version,
                                    hw_version,
                                } = data
                                {
                                    // Read only once
                                    //device_model.serial = serial;
                                    device_model.fw_version = fw_version;
                                    device_model.hw_version = hw_version;
                                }
                            }
                            Commands::DEVICE_POSITION => {
                                if let ProtocolData::Position { value } = data {
                                    device_model.position.reported_value = value;
                                }
                            }
                            Commands::DEVICE_STATUS => {
                                if let ProtocolData::Status(status) = data {
                                    device_model.status = status;
                                }
                            }
                            Commands::DEVICE_ANALOG => {
                                if let ProtocolData::Analog { a0, a1 } = data {
                                    device_model.analog0.reported_value = a0;
                                    device_model.analog1.reported_value = a1;
                                }
                            }
                            Commands::DEVICE_SERVO => {
                                if let ProtocolData::Servo { s0, s1 } = data {
                                    device_model.servo0.reported_value = s0;
                                    device_model.servo1.reported_value = s1;
                                }
                            }
                            Commands::DEVICE_LED => {
                                if let ProtocolData::Led(led) = data {
                                    device_model.led_stat.reported_value = led.stat;
                                    device_model.led_sys.reported_value = led.sys;
                                }
                            }
                            Commands::DEVICE_MOTOR => {
                                if let ProtocolData::Motor(motor) = data {
                                    device_model.motor = motor;
                                }
                            }
                            Commands::DEVICE_FOC => {
                                if let ProtocolData::FOC {
                                    foc_1,
                                    foc_2,
                                    foc_3,
                                    en,
                                } = data
                                {
                                    device_model.foc1.reported_value = foc_1;
                                    device_model.foc2.reported_value = foc_2;
                                    device_model.foc3.reported_value = foc_3;
                                    device_model.foc_en.reported_value = en;
                                }
                            }
                            _ => {
                                // unimplemented
                                println!("Warning: Unhandled device command: {:?}", cmd);
                            }
                        }
                    }
                } else {
                    println!("Error: Undiscovered device: {:?}", id);
                }
            }
        }
    }

    // Handles messages until a discriminant matches
    pub async fn wait_for_message(
        &mut self,
        predicate: impl Fn(&CanDriveMessage) -> bool,
    ) -> Result<CanDriveMessage::Commands, String> {
        tokio::select! {
            self.cancellation_token.cancelled() => {
                return Err("Cancelled".to_string())
            },
            tokio::time::delay_for(self.timeout) {
                return Err("Timeout".to_string())
            },
            {
                loop {
                    if let Some(msg) = self.can_txrx.recv().await {
                        if predicate(msg)
                        {
                            return Ok(msg);
                        } else {
                            self.default_message_handler(msg);
                        }
                    }
                }
            },
        };
    }

    pub async fn transaction(
        &mut self,
        id: DeviceId,
        cmd: Commands,
        data: ProtocolData,
    ) -> Result<(), &'static str> {
        if let Some(mut device_model) = self.get_device_mut(id) {
            self.can_txrx
                .send(CanDriveMessage::Control { id, cmd, data })
                .await?;
            match self.wait_for_message(|msg| -> bool {
                match msg {
                    CanDriveMessage::Control {
                        ack_id,
                        ack_cmd,
                        data,
                    } => {
                        if msg.is_device_command() {
                            // Check for ack integrity
                            if ack_id == id && ack_cmd == cmd {
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
                    self.default_message_handler(msg).await
                }
                Err(e) => Err(e), // Don't rewind device assignment, just use the next one
            }
        } else {
            Err("Device not found")
        }
    }

    pub fn start(&mut self) -> Vec<tokio::JoinHandle<()>> {
        let threads = Vec::new();
        // Handle local commands
        threads.push(tokio::spawn(async move {
            loop {
                tokio::select! {
                    _ = self.cancellation_token.cancelled() => {
                        break;
                    },
                    lcmd = self.local_txrx.recv() => {
                        if let Some(lcmd) = lcmd {
                            match lcmd {
                                LocalCommand::ListDevices => {
                                   self.devices.iter().for_each(|(id, device)| {
                                       println!("Device ID: {:?},\n\tSerial: {:?}\n\tFW: {:?}\n\tHW: {:?}", id, device.serial, device.fw_version, device.hw_version);
                                   });
                                }
                                _ => {}
                            }
                        }
                    },
                };
            }
        }));
        // Handle CAN device messages
        threads.push(tokio::spawn(async move {
            loop {
                tokio::select! {
                    _ = self.cancellation_token.cancelled() => {
                        break;
                    },
                    data = self.can_txrx.recv() => {
                        if let Some(data) = data {
                            self.default_message_handler(data).await;
                        }
                    },
                };
            }
        }));
        return threads;
    }
}
