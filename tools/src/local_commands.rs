use std::fmt::{Display, Formatter};
use std::str::FromStr;
use std::{hash::Hash, io::BufRead};
use std::thread::JoinHandle;
use crossbeam_channel::{Receiver, unbounded};
use strum_macros::{EnumDiscriminants, EnumIter};

use crate::protocol::{ DeviceId };
use crate::util::{Threaded, critical_error, is_running, set_running};
use log::{error, info, trace};

#[repr(u8)]
#[derive(Clone, Debug, Default, EnumDiscriminants)]
#[strum_discriminants(derive(EnumIter))]
#[strum_discriminants(name(LocalCommandTypes))]
pub enum LocalCommands {
    #[default]
    ListDevices, // Dump local device cache to stdout
    ResetNetwork, // Broadcast network reset (devices start discovery again)
    SetPosition(DeviceId, u16),
    GetPosition(DeviceId),
    GetInfo(DeviceId),
    GetStatus(DeviceId),
    SetSysLed(DeviceId, u8),
    SetStatLed(DeviceId, u8),
    GetLeds(DeviceId),
    GetMotor(DeviceId),
    GetAnalog(DeviceId),
    GetServo(DeviceId),
    SetServo0(DeviceId, u16),
    SetServo1(DeviceId, u16),
    UpdateFirmware(DeviceId, String),
    RevokeConfig(DeviceId),
    Help,
    Exit,
}

impl Hash for LocalCommands {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        state.write_u8(self.index());
        match self {
            LocalCommands::SetPosition(id, pos) => {
                state.write_u16(id.to_can_id());
                state.write_u16(*pos);
            },
            LocalCommands::GetPosition(id) => {
                state.write_u16(id.to_can_id());
            },
            LocalCommands::GetInfo(id) => {
                state.write_u16(id.to_can_id());
            },
            LocalCommands::GetStatus(id) => {
                state.write_u16(id.to_can_id());
            },
            LocalCommands::SetSysLed(id, v) => {
                state.write_u16(id.to_can_id());
                state.write_u8(*v);
            },
            LocalCommands::SetStatLed(id, v) => {
                state.write_u16(id.to_can_id());
                state.write_u8(*v);
            },
            LocalCommands::GetLeds(id) => {
                state.write_u16(id.to_can_id());
            },
            LocalCommands::GetMotor(id) => {
                state.write_u16(id.to_can_id());
            },
            LocalCommands::GetAnalog(id) => {
                state.write_u16(id.to_can_id());
            },
            LocalCommands::GetServo(id) => {
                state.write_u16(id.to_can_id());
            },
            LocalCommands::SetServo0(id, pos) => {
                state.write_u16(id.to_can_id());
                state.write_u16(*pos);
            },
            LocalCommands::SetServo1(id, pos) => {
                state.write_u16(id.to_can_id());
                state.write_u16(*pos);
            },
            LocalCommands::UpdateFirmware(id, path) => {
                state.write_u16(id.to_can_id());
                state.write(path.as_bytes());
            },
            LocalCommands::RevokeConfig(id) => {
                state.write_u16(id.to_can_id());
            },
            LocalCommands::Help => state.write_u8(16),
            LocalCommands::Exit => state.write_u8(15),
            _ => (),
        }
    }
}

impl PartialEq for LocalCommands {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (LocalCommands::ListDevices, LocalCommands::ListDevices) => true,
            (LocalCommands::ResetNetwork, LocalCommands::ResetNetwork) => true,
            (LocalCommands::SetPosition(id1, pos1), LocalCommands::SetPosition(id2, pos2)) => id1 == id2 && pos1 == pos2,
            (LocalCommands::GetPosition(id1), LocalCommands::GetPosition(id2)) => id1 == id2,
            (LocalCommands::GetInfo(id1), LocalCommands::GetInfo(id2)) => id1 == id2,
            (LocalCommands::GetStatus(id1), LocalCommands::GetStatus(id2)) => id1 == id2,
            (LocalCommands::SetSysLed(id1, state1), LocalCommands::SetSysLed(id2, state2)) => id1 == id2 && state1 == state2,
            (LocalCommands::SetStatLed(id1, state1), LocalCommands::SetStatLed(id2, state2)) => id1 == id2 && state1 == state2,
            (LocalCommands::GetLeds(id1), LocalCommands::GetLeds(id2, )) => id1 == id2,
            (LocalCommands::GetMotor(id1), LocalCommands::GetMotor(id2)) => id1 == id2,
            (LocalCommands::GetAnalog(id1), LocalCommands::GetAnalog(id2)) => id1 == id2,
            (LocalCommands::GetServo(id1), LocalCommands::GetServo(id2)) => id1 == id2,
            (LocalCommands::SetServo0(id1, pos1), LocalCommands::SetServo0(id2, pos2)) => id1 == id2 && pos1 == pos2,
            (LocalCommands::SetServo1(id1, pos1), LocalCommands::SetServo1(id2, pos2)) => id1 == id2 && pos1 == pos2,
            (LocalCommands::UpdateFirmware(id1, path1), LocalCommands::UpdateFirmware(id2, path2)) => id1 == id2 && path1 == path2,
            (LocalCommands::RevokeConfig(id1), LocalCommands::RevokeConfig(id2)) => id1 == id2,
            (LocalCommands::Help, LocalCommands::Help) => true,
            (LocalCommands::Exit, LocalCommands::Exit) => true,
            _ => false,
        }
    }
}

impl Eq for LocalCommands {}

impl Display for LocalCommands {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            LocalCommands::ListDevices => write!(f, "list"),
            LocalCommands::ResetNetwork => write!(f, "reset_network"),
            LocalCommands::SetPosition(id, pos) => write!(f, "set_position {:X} {}", id, pos),
            LocalCommands::GetPosition(id) => write!(f, "get_position {:X}", id),
            LocalCommands::GetInfo(id) => write!(f, "get_info {:X}", id),
            LocalCommands::GetStatus(id) => write!(f, "get_status {:X}", id),
            LocalCommands::SetSysLed(id, state) => write!(f, "set_sys_led {:X} {}", id, state),
            LocalCommands::SetStatLed(id, state) => write!(f, "set_stat_led {:X} {}", id, state),
            LocalCommands::GetLeds(id) => write!(f, "get_led {:X}", id),
            LocalCommands::GetMotor(id) => write!(f, "get_motor {:X}", id),
            LocalCommands::GetAnalog(id) => write!(f, "get_analog {:X}", id),
            LocalCommands::GetServo(id) => write!(f, "get_servo {:X}", id),
            LocalCommands::SetServo0(id, pos) => write!(f, "set_servo0 {:X} {}", id, pos),
            LocalCommands::SetServo1(id, pos) => write!(f, "set_servo1 {:X} {}", id, pos),
            LocalCommands::UpdateFirmware(id, path) => write!(f, "update_firmware {:X} {}", id, path),
            LocalCommands::RevokeConfig(id) => write!(f, "revoke_config {:X}", id),
            LocalCommands::Help => write!(f, "help"),
            LocalCommands::Exit => write!(f, "exit"),
        }
    }
}

impl FromStr for LocalCommands {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let mut args = s.split_whitespace();
        match args.next().ok_or("Missing command")? {
            "list" => Ok(LocalCommands::ListDevices),
            "reset_network" => Ok(LocalCommands::ResetNetwork),
            "set_position" => {
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                let pos = args.next().ok_or("Missing position")?.parse::<u16>().map_err(|e| e.to_string())?;
                Ok(LocalCommands::SetPosition(id, pos))
            },
            "get_position" => {
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                Ok(LocalCommands::GetPosition(id))
            },
            "get_info" => {
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                Ok(LocalCommands::GetInfo(id))
            },
            "get_status" => {
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                Ok(LocalCommands::GetStatus(id))
            },
            "set_sys_led" => {
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                let state = args.next().ok_or("Missing state")?.parse::<u8>().map_err(|e| e.to_string())?;
                Ok(LocalCommands::SetSysLed(id, state))
            },
            "set_stat_led" => {
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                let state = args.next().ok_or("Missing state")?.parse::<u8>().map_err(|e| e.to_string())?;
                Ok(LocalCommands::SetStatLed(id, state))
            },
            "get_led" => {
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                Ok(LocalCommands::GetLeds(id))
            },
            "get_motor" => {
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                Ok(LocalCommands::GetMotor(id))
            },
            "get_analog" => {
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                Ok(LocalCommands::GetAnalog(id))
            },
            "get_servo" => {
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                Ok(LocalCommands::GetServo(id))
            },
            "set_servo0" => {
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                let pos = args.next().ok_or("Missing position")?.parse::<u16>().map_err(|e| e.to_string())?;
                Ok(LocalCommands::SetServo0(id, pos))
            },
            "set_servo1" => {
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                let pos = args.next().ok_or("Missing position")?.parse::<u16>().map_err(|e| e.to_string())?;
                Ok(LocalCommands::SetServo1(id, pos))
            },
            "update_firmware" => {
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                let path = args.next().ok_or("Missing path")?.to_string();
                Ok(LocalCommands::UpdateFirmware(id, path))
            },
            "revoke_config" => {
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                Ok(LocalCommands::RevokeConfig(id))
            },
            "help" => Ok(LocalCommands::Help),
            "exit" => Ok(LocalCommands::Exit),
            _ => Err("Unknown command".to_string()),
        }
    }
}


impl LocalCommands {
    pub fn index(&self) -> u8 {
         // SAFETY: Because `Self` is marked `repr(u8)`, its layout is a `repr(C)` `union`
        // between `repr(C)` structs, each of which has the `u8` discriminant as its first
        // field, so we can read the discriminant without offsetting the pointer.
        unsafe { *<*const _>::from(self).cast::<u8>() }
    }
}

pub struct LocalCommandParser
{
    line_queue: Receiver<LocalCommands>,
    thread: Option<JoinHandle<()>>,
}

impl LocalCommandParser
{
    pub fn new() -> Self
    {
        trace!("Starting local command parser");
        let (tx, rx) = unbounded();
        let th = std::thread::spawn(move ||
        {
            trace!("local command parser thread entered");
            let mut stdin = std::io::stdin().lock();
            let mut buffer = String::new();
            while is_running() {
                info!("Enter command>");
                match stdin.read_line(&mut buffer)
                {
                    Ok(byte_count) => {
                        if byte_count > 0 {
                            trace!("Received input: {}", buffer);
                            match LocalCommands::from_str(&buffer)
                            {
                                Ok(cmd) => {
                                    info!("Parsed command: {:?}", cmd);
                                    match tx.send(cmd) {
                                        Ok(_) => (),
                                        Err(e) => {
                                            critical_error(format!("Error sending to stdin queue: {}", e));
                                            break;
                                        }
                                    }
                                }
                                Err(e) => error!("Error parsing command: {}", e),
                            };
                        } else {
                            return; // EOF
                        }
                    },
                    Err(e) => {
                        critical_error(format!("Error reading from stdin: {}", e));
                        break;
                    }
                }
                buffer.clear();
            }
        });
        LocalCommandParser {
            line_queue: rx,
            thread: Some(th),
        }
    }

    pub fn get_command_queue(&self) -> Receiver<LocalCommands> {
        return self.line_queue.clone();
    }

    pub fn default_handler(&self, message: LocalCommands) -> Result<(), String>
    {
        // Handle command
        trace!("Invoked Default Message Handler: Local command: {}", message);
        match message {
            LocalCommands::Help => {
                info!("Available commands:");
                info!("  list");
                info!("  reset_network");
                info!("  set_position <device_id> <position>");
                info!("  get_position <device_id>");
                info!("  get_info <device_id>");
                info!("  get_status <device_id>");
                info!("  set_sys_led <device_id> <state>");
                info!("  set_stat_led <device_id> <state>");
                info!("  get_led <device_id>");
                info!("  get_motor <device_id>");
                info!("  get_analog <device_id>");
                info!("  get_servo <device_id>");
                info!("  set_servo0 <device_id> <position>");
                info!("  set_servo1 <device_id> <position>");
                info!("  update_firmware <device_id> <path>");
                info!("  revoke_config <device_id>");
                info!("  exit");
                Ok(())
            },
            LocalCommands::Exit => {
                set_running(false);
                Ok(())
            },
            _ => Ok(())
        }
    }
}

impl Threaded for LocalCommandParser {
    fn join(&mut self) {
        self.thread.take().unwrap().join().unwrap();
    }
}