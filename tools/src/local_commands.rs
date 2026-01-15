use std::fmt::{Display, Formatter};
use std::str::FromStr;
use std::{hash::Hash, io::BufRead};
use std::thread::JoinHandle;
use crossbeam_channel::{Receiver, Sender, unbounded};

use crate::protocol::{CanDriveMessage, Commands, DeviceId, ProtocolData};
use crate::util::Threaded;

#[derive(Clone, Debug, Default)]
pub enum LocalCommands {
    #[default]
    ListDevices, // Dump local device cache to stdout
    ResetNetwork, // Broadcast network reset (devices start discovery again)
    SetPosition(DeviceId, u16),
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
    Exit,
}

impl Hash for LocalCommands {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        match self {
            LocalCommands::ListDevices => state.write_u8(1),
            LocalCommands::ResetNetwork => state.write_u8(2),
            LocalCommands::SetPosition(id, pos) => {
                state.write_u8(3);
                state.write_u16(id.to_can_id());
                state.write_u16(*pos);
            },
            LocalCommands::GetInfo(id) => {
                state.write_u8(4);
                state.write_u16(id.to_can_id());
            },
            LocalCommands::GetStatus(id) => {
                state.write_u8(5);
                state.write_u16(id.to_can_id());
            },
            LocalCommands::SetSysLed(id, v) => {
                state.write_u8(6);
                state.write_u16(id.to_can_id());
                state.write_u8(*v);
            },
            LocalCommands::SetStatLed(id, v) => {
                state.write_u8(7);
                state.write_u16(id.to_can_id());
                state.write_u8(*v);
            },
            LocalCommands::GetLeds(id) => {
                state.write_u8(8);
                state.write_u16(id.to_can_id());
            },
            LocalCommands::GetMotor(id) => {
                state.write_u8(9);
                state.write_u16(id.to_can_id());
            },
            LocalCommands::GetAnalog(id) => {
                state.write_u8(10);
                state.write_u16(id.to_can_id());
            },
            LocalCommands::GetServo(id) => {
                state.write_u8(11);
                state.write_u16(id.to_can_id());
            },
            LocalCommands::SetServo0(id, pos) => {
                state.write_u8(12);
                state.write_u16(id.to_can_id());
                state.write_u16(*pos);
            },
            LocalCommands::SetServo1(id, pos) => {
                state.write_u8(13);
                state.write_u16(id.to_can_id());
                state.write_u16(*pos);
            },
            LocalCommands::UpdateFirmware(id, path) => {
                state.write_u8(14);
                state.write_u16(id.to_can_id());
                state.write(path.as_bytes());
            },
            LocalCommands::Exit => state.write_u8(15),
        }
    }
}

impl PartialEq for LocalCommands {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (LocalCommands::ListDevices, LocalCommands::ListDevices) => true,
            (LocalCommands::ResetNetwork, LocalCommands::ResetNetwork) => true,
            (LocalCommands::SetPosition(id1, pos1), LocalCommands::SetPosition(id2, pos2)) => id1 == id2 && pos1 == pos2,
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
            LocalCommands::SetPosition(id, pos) => write!(f, "set_position {} {}", id, pos),
            LocalCommands::GetInfo(id) => write!(f, "get_info {}", id),
            LocalCommands::GetStatus(id) => write!(f, "get_status {}", id),
            LocalCommands::SetSysLed(id, state) => write!(f, "set_sys_led {} {}", id, state),
            LocalCommands::SetStatLed(id, state) => write!(f, "set_stat_led {} {}", id, state),
            LocalCommands::GetLeds(id) => write!(f, "get_led {}", id),
            LocalCommands::GetMotor(id) => write!(f, "get_motor {}", id),
            LocalCommands::GetAnalog(id) => write!(f, "get_analog {}", id),
            LocalCommands::GetServo(id) => write!(f, "get_servo {}", id),
            LocalCommands::SetServo0(id, pos) => write!(f, "set_servo0 {} {}", id, pos),
            LocalCommands::SetServo1(id, pos) => write!(f, "set_servo1 {} {}", id, pos),
            LocalCommands::UpdateFirmware(id, path) => write!(f, "update_firmware {} {}", id, path),
            LocalCommands::Exit => write!(f, "exit"),
        }
    }
}

impl FromStr for LocalCommands {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "list" => Ok(LocalCommands::ListDevices),
            "reset_network" => Ok(LocalCommands::ResetNetwork),
            "set_position" => {
                let mut args = s.split_whitespace();
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                let pos = args.next().ok_or("Missing position")?.parse::<u16>().map_err(|e| e.to_string())?;
                Ok(LocalCommands::SetPosition(id, pos))
            },
            "get_info" => {
                let mut args = s.split_whitespace();
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                Ok(LocalCommands::GetInfo(id))
            },
            "get_status" => {
                let mut args = s.split_whitespace();
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                Ok(LocalCommands::GetStatus(id))
            },
            "set_sys_led" => {
                let mut args = s.split_whitespace();
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                let state = args.next().ok_or("Missing state")?.parse::<u8>().map_err(|e| e.to_string())?;
                Ok(LocalCommands::SetSysLed(id, state))
            },
            "set_stat_led" => {
                let mut args = s.split_whitespace();
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                let state = args.next().ok_or("Missing state")?.parse::<u8>().map_err(|e| e.to_string())?;
                Ok(LocalCommands::SetStatLed(id, state))
            },
            "get_led" => {
                let mut args = s.split_whitespace();
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                Ok(LocalCommands::GetLeds(id))
            },
            "get_motor" => {
                let mut args = s.split_whitespace();
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                Ok(LocalCommands::GetMotor(id))
            },
            "get_analog" => {
                let mut args = s.split_whitespace();
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                Ok(LocalCommands::GetAnalog(id))
            },
            "get_servo" => {
                let mut args = s.split_whitespace();
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                Ok(LocalCommands::GetServo(id))
            },
            "set_servo0" => {
                let mut args = s.split_whitespace();
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                let pos = args.next().ok_or("Missing position")?.parse::<u16>().map_err(|e| e.to_string())?;
                Ok(LocalCommands::SetServo0(id, pos))
            },
            "set_servo1" => {
                let mut args = s.split_whitespace();
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                let pos = args.next().ok_or("Missing position")?.parse::<u16>().map_err(|e| e.to_string())?;
                Ok(LocalCommands::SetServo1(id, pos))
            },
            "update_firmware" => {
                let mut args = s.split_whitespace();
                let id = DeviceId::from_str(args.next().ok_or("Missing device id")?)?;
                let path = args.next().ok_or("Missing path")?.to_string();
                Ok(LocalCommands::UpdateFirmware(id, path))
            },
            "exit" => Ok(LocalCommands::Exit),
            _ => Err("Unknown command".to_string()),
        }
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
        let (tx, rx) = unbounded();
        let th = std::thread::spawn(move ||
        {
            let mut stdin = std::io::stdin().lock();
            let mut buffer = String::new();
            loop {
                match stdin.read_line(&mut buffer)
                {
                    Ok(byte_count) => {
                        if byte_count > 0 {
                            match LocalCommands::from_str(&buffer)
                            {
                                Ok(cmd) => {
                                    match tx.send(cmd) {
                                        Ok(_) => (),
                                        Err(e) => {
                                            eprintln!("Error sending to stdin queue: {}", e);
                                            break;
                                        }
                                    }
                                }
                                Err(e) => eprintln!("Error parsing command: {}", e),
                            };
                        } else {
                            return; // EOF
                        }
                    },
                    Err(e) => {
                        eprintln!("Error reading from stdin: {}", e);
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

    pub fn default_handler(&self, message: LocalCommands, can_tx: &Sender<CanDriveMessage>) -> Result<(), String>
    {
        // Handle command
        println!("Local command: {}", message);
        match message {
            LocalCommands::Exit => {
                println!("Unhandeled exit command");
                Ok(())
            },
            LocalCommands::ResetNetwork => {
                println!("Broadcasting network reset...");
                can_tx.send(
                    CanDriveMessage::Control {
                        id: DeviceId::new(0),
                        is_controller: true,
                        cmd: Commands::NETWORK_RESET,
                        data: ProtocolData::Empty,
                    })
                    .map_err(|e| { format!("{}", e) })
            },
            LocalCommands::ListDevices => {
                println!("Unhandled list command");
                Ok(())
            },
            LocalCommands::SetPosition(id, pos) => {
                let msg = CanDriveMessage::Control {
                    id: id,
                    is_controller: true,
                    cmd: Commands::SET_POSITION,
                    data: ProtocolData::Position{ value: pos}
                };
                can_tx.send(msg).map_err(|e| format!("{}", e))
            },
            LocalCommands::GetInfo(id) => {
                let msg = CanDriveMessage::Control {
                    id: id,
                    is_controller: true,
                    cmd: Commands::GET_INFO,
                    data: ProtocolData::Empty
                };
                can_tx.send(msg).map_err(|e| format!("{}", e))
            },
            LocalCommands::GetStatus(id) => {
                let msg = CanDriveMessage::Control {
                    id: id,
                    is_controller: true,
                    cmd: Commands::GET_STATUS,
                    data: ProtocolData::Empty
                };
                can_tx.send(msg).map_err(|e| format!("{}", e))
            },
            LocalCommands::SetSysLed(id, value) => {
                let msg = CanDriveMessage::Control {
                    id: id,
                    is_controller: true,
                    cmd: Commands::SET_LED,
                    data: ProtocolData::Led {
                        sys: value,
                        stat: 0,
                        update_flag: 0x01
                    }
                };
                can_tx.send(msg).map_err(|e| format!("{}", e))
            },
            LocalCommands::SetStatLed(id, value) => {
                let msg = CanDriveMessage::Control {
                    id: id,
                    is_controller: true,
                    cmd: Commands::SET_LED,
                    data: ProtocolData::Led {
                        sys: 0,
                        stat: value,
                        update_flag: 0x02
                    }
                };
                can_tx.send(msg).map_err(|e| format!("{}", e))
            }
            LocalCommands::GetLeds(id) => {
                let msg = CanDriveMessage::Control {
                    id: id,
                    is_controller: true,
                    cmd: Commands::GET_LED,
                    data: ProtocolData::Empty
                };
                can_tx.send(msg).map_err(|e| format!("{}", e))
            },
            LocalCommands::GetMotor(id) => {
                let msg = CanDriveMessage::Control {
                    id: id,
                    is_controller: true,
                    cmd: Commands::GET_MOTOR,
                    data: ProtocolData::Empty
                };
                can_tx.send(msg).map_err(|e| format!("{}", e))
            },
            LocalCommands::GetAnalog(id) => {

                let msg = CanDriveMessage::Control {
                    id: id,
                    is_controller: true,
                    cmd: Commands::GET_ANALOG,
                    data: ProtocolData::Empty
                };

                can_tx.send(msg).map_err(|e| format!("{}", e))
            },
            LocalCommands::GetServo(id) => {

                let msg = CanDriveMessage::Control{
                    id: id,
                    is_controller: true,
                    cmd: Commands::GET_SERVO,
                    data: ProtocolData::Empty
                };
                can_tx.send(msg).map_err(|e| format!("{}", e))
            },
            LocalCommands::SetServo0(id, pos) => {
                // Send command to device via comm_manager
                let msg = CanDriveMessage::Control {
                    id: id,
                    is_controller: true,
                    cmd: Commands::SET_SERVO,
                    data: ProtocolData::Servo { s0: pos, s1: 0, update_flag: 0x01 }
                };
                can_tx.send(msg).map_err(|e| format!("{}", e))
            },
            LocalCommands::SetServo1(id, pos) => {
                // Send command to device via comm_manager
                let msg = CanDriveMessage::Control {
                    id: id,
                    is_controller: true,
                    cmd: Commands::SET_SERVO,
                    data: ProtocolData::Servo { s0: 0, s1: pos, update_flag: 0x02 }
                };
                can_tx.send(msg).map_err(|e| format!("{}", e))
            },
            LocalCommands::UpdateFirmware(_id, _path) => {
                Err("Update not implemented yet".to_string())
            },
            _ => {
                Err("Command not implemented yet".to_string())
            }
        }
    }
}

impl Threaded for LocalCommandParser {
    fn join(&mut self) {
        self.thread.take().unwrap().join().unwrap();
    }
}