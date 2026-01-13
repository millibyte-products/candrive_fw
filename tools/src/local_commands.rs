use std::{collections::{HashSet, Vec}, hash::Hash};
use crate::protocol::DeviceId;

pub enum LocalCommands {
    ListDevices, // Dump local device cache to stdout
    ResetNetwork, // Broadcast network reset (devices start discovery again)
    SetPosition,
    GetInfo,
    GetStatus,
    SetSysLed,
    SetStatLed,
    GetMotor,
    GetAnalog,
    GetServo,
    SetServo0,
    SetServo1,
    UpdateFirmware, // DeviceId, path to firmware file
    Exit,
}

pub enum LocalArguments {
    DeviceId,
    Position,
    Path,
    LedState,
}

pub enum ParsedArguments {
    DeviceId(DeviceId),
    Position16(u16),
    Position8(u8),
    Path(String),
    LedState(bool),
}

struct LocalCommand
{
    command: LocalCommands,
    args: Vec<LocalArguments>,
}

impl LocalCommand
{
    pub fn new(command: LocalCommands,aliases: &[&str], args: Vec<LocalArguments>) -> Self
    {
        LocalCommand { command, args }
    }
}

struct ParsedCommand
{
    command: LocalCommands,
    args: Vec<ParsedArguments>,
}

struct LocalCommandParser
{
    aliases: HashSet<String, LocalCommands>,
}

impl LocalCommandParser
{
    pub fn new() -> Self
    {
        LocalCommandParser {
            aliases: HashSet::<String, LocalCommands>::new(),
        }
    }

    pub fn default() -> Self
    {
        let mut this = LocalCommandParser::new();
        this.def(LocalCommands::ListDevices, &["list", "ls"], vec![]).unwrap();
        this.def(LocalCommands::ResetNetwork, &["reset_network", "reset_all"], vec![]).unwrap();
        this.def(LocalCommands::SetPosition, &["set_position", "move_to", "p"], vec![LocalArguments::DeviceId, LocalArguments::Position16]).unwrap();
        this.def(LocalCommands::GetInfo, &["get_info", "info", "i"], vec![LocalArguments::DeviceId]).unwrap();
        this.def(LocalCommands::GetStatus, &["get_status", "status", "s"], vec![LocalArguments::DeviceId]).unwrap();
        this.def(LocalCommands::SetSysLed, &["set_sys_led", "sys_led"], vec![LocalArguments::DeviceId, LocalArguments::LedState]).unwrap();
        this.def(LocalCommands::SetStatLed, &["set_stat_led", "stat_led"], vec![LocalArguments::DeviceId, LocalArguments::LedState]).unwrap();
        this.def(LocalCommands::GetMotor, &["get_motor", "motor", "m"], vec![LocalArguments::DeviceId]).unwrap();
        this.def(LocalCommands::GetAnalog, &["get_analog", "analog", "a"], vec![LocalArguments::DeviceId]).unwrap();
        this.def(LocalCommands::GetServo, &["get_servo", "servo", "v"], vec![LocalArguments::DeviceId]).unwrap();
        this.def(LocalCommands::SetServo0, &["set_servo0", "servo0", "v0"], vec![LocalArguments::DeviceId, LocalArguments::Position8]).unwrap();
        this.def(LocalCommands::SetServo1, &["set_servo1", "servo1", "v1"], vec![LocalArguments::DeviceId, LocalArguments::Position8]).unwrap();
        this.def(LocalCommands::UpdateFirmware, &["update_firmware", "fw_update"], vec![LocalArguments::DeviceId, LocalArguments::Path]).unwrap();
        this.def(LocalCommands::Exit, &["exit", "quit", "q"], vec![]).unwrap();
        this
    }

    pub fn def(&mut self, command: LocalCommands, aliases: &[&str], args: Vec<LocalArguments>) -> Result<(), String>
    {
        for alias in aliases.iter()
        {
            if (self.aliases.contains(aliases)) {
                return Err("Alias already defined".to_string());
            }
            self.aliases.insert(alias.to_string(), command.clone());
        }
        Ok(())
    }

    pub fn parse(&self, input: String) -> Result<ParsedCommand, String>
    {
        let mut args = input.split_whitespace();
        if let Some(command_str) = args.next() {
            if let Some(command) = self.aliases.get(command_str) {
                let mut parsed_command = ParsedCommand {
                    command: command.command.clone(),
                    args: Vec::new(),
                };
                for arg in command.args.iter() {
                    if let Some(_next_arg) = args.next() {
                        // Parse argument based on type
                        match(arg) {
                            LocalArguments::DeviceId => {
                                // Parse DeviceId
                                parsed_command.args.push(ParsedArguments::DeviceId(DeviceId::from_str(_next_arg).map_err(|e| e.to_string())?));
                            }
                            LocalArguments::Position16 => {
                                // Parse u16 position
                                parsed_command.args.push(ParsedArguments::Position16(_next_arg.parse::<u16>().map_err(|e| e.to_string())?));
                            }
                            LocalArguments::Position8 => {
                                // Parse u8 position
                                parsed_command.args.push(ParsedArguments::Position8(_next_arg.parse::<u8>().map_err(|e| e.to_string())?));
                            }
                            LocalArguments::Path => {
                                // Parse path string
                                parsed_command.args.push(ParsedArguments::Path(_next_arg.to_string()));
                            }
                            LocalArguments::LedState => {
                                // Parse bool led state
                                parsed_command.args.push(ParsedArguments::LedState(match _next_arg.to_lowercase().as_str() {
                                    "true" | "1" | "on" => true,
                                    "false" | "0" | "off" => false,
                                    _ => return Err("Invalid value for LedState, expected true/false".to_string()),
                                }));
                            }
                            _ => {
                                return Err("Unknown argument type".to_string());
                            }
                        }
                        return Ok((parsed_command))
                    } else {
                        return Err(format!("Missing argument for command {:?}", command));
                    }
                }
            } else {
                return Err("Unknown command".to_string());
            }
        }
        Err("No command provided".to_string())
    }

    pub async fn handle_command(&self, parsed_command: ParsedCommand, cancellation_token: CancellationToken, can_tx: &Sender<CanDriveMessage>, device_tx: &Sender<LocalCommand>) -> Result<(), String>
    {
        // Handle command
        println!("Parsed command: {:?}", parsed_command);
        match parsed_command.command {
            LocalCommands::Exit => {
                println!("Exiting...");
                cancellation_token.cancel();
            },
            LocalCommands::ResetNetwork => {
                println!("Broadcasting network reset...");
                can_tx.send(
                    CanDriveMessage::new(
                        0,
                        Commands::CONTROLLER_NETWORK_RESET,
                        ProtocolData::None,
                    )).unwrap();
            },
            LocalCommands::ListDevices => {
                // Forward command
                device_tx.send(
                    LocalCommand::new(
                        LocalCommands::ListDevices,
                        &[],
                        vec![],
                    )
                ).unwrap();
            },
            LocalCommands::SetPosition => {
                // Example handling of SetPosition command
                let mut device_id: Option<DeviceId> = None;
                let mut position: Option<u16> = None;
                for arg in parsed_command.args {
                    match arg {
                        ParsedArguments::DeviceId(id) => {
                            device_id = Some(id);
                        },
                        ParsedArguments::Position16(pos) => {
                            position = Some(pos);
                        },
                        _ => {}
                    }
                }
                if let (Some(id), Some(pos)) = (device_id, position) {
                    println!("Setting position of device {:?} to {}", id, pos);
                    // Send command to device via comm_manager
                    let msg = CanDriveMessage::Control(
                        id,
                        Commands::SET_POSITION,
                        ProtocolData::Position{value: pos}
                    );
                    broadcast_tx.send(msg).unwrap();
                } else {
                    println!("Error: Missing arguments for SetPosition command");
                }
            },
            LocalCommands::GetInfo => {
                let mut device_id: Option<DeviceId> = None;
                for arg in parsed_command.args {
                    match arg {
                        ParsedArguments::DeviceId(id) => {
                            device_id = Some(id);
                        },
                        _ => {}
                    }
                }
                if let Some(id) = device_id {
                    println!("Requesting info from device {:?}", id);
                    let msg = CanDriveMessage::Control(
                        id,
                        Commands::GET_INFO,
                        ProtocolData::None
                    );
                    can_tx.send(msg).unwrap();
                } else {
                    println!("Error: Missing DeviceId argument for GetInfo command");
                }
            },
            LocalCommands::GetStatus => {
                let mut device_id: Option<DeviceId> = None;
                for arg in parsed_command.args {
                    match arg {
                        ParsedArguments::DeviceId(id) => {
                            device_id = Some(id);
                        },
                        _ => {}
                    }
                }
                if let Some(id) = device_id {
                    println!("Requesting status from device {:?}", id);
                    let msg = CanDriveMessage::Control(
                        id,
                        Commands::GET_STATUS,
                        ProtocolData::None
                    );
                    can_tx.send(msg).unwrap();
                } else {
                    println!("Error: Missing DeviceId argument for GetStatus command");
                }
            },
            LocalCommands::SetSysLed => {
                let mut device_id: Option<DeviceId> = None;
                let mut led_state: Option<bool> = None;
                for arg in parsed_command.args {
                    match arg {
                        ParsedArguments::DeviceId(id) => {
                            device_id = Some(id);
                        },
                        ParsedArguments::LedState(state) => {
                            led_state = Some(state);
                        },
                        _ => {}
                    }
                }
                if let (Some(id), Some(state)) = (device_id, led_state) {
                    println!("Setting system LED of device {:?} to {}", id, state);
                    let msg = CanDriveMessage::Control(
                        id,
                        Commands::SET_SYS_LED,
                        ProtocolData::LedState{state}
                    );
                    can_tx.send(msg).unwrap();
                } else {
                    println!("Error: Missing arguments for SetSysLed command");
                }
            },
            LocalCommands::SetStatLed => {
                let mut device_id: Option<DeviceId> = None;
                let mut led_state: Option<bool> = None;
                for arg in parsed_command.args {
                    match arg {
                        ParsedArguments::DeviceId(id) => {
                            device_id = Some(id);
                        },
                        ParsedArguments::LedState(state) => {
                            led_state = Some(state);
                        },
                        _ => {}
                    }
                }
                if let (Some(id), Some(state)) = (device_id, led_state) {
                    println!("Setting status LED of device {:?} to {}", id, state);
                    let msg = CanDriveMessage::Control(
                        id,
                        Commands::SET_STAT_LED,
                        ProtocolData::LedState{state}
                    );
                    can_tx.send(msg).unwrap();
                } else {
                    println!("Error: Missing arguments for SetStatLed command");
                }
            },
            LocalCommands::GetMotor => {
                let mut device_id: Option<DeviceId> = None;
                for arg in parsed_command.args {
                    match arg {
                        ParsedArguments::DeviceId(id) => {
                            device_id = Some(id);
                        },
                        _ => {}
                    }
                }
                if let Some(id) = device_id {
                    println!("Requesting motor data from device {:?}", id);
                    let msg = CanDriveMessage::Control(
                        id,
                        Commands::GET_MOTOR,
                        ProtocolData::None
                    );
                    can_tx.send(msg).unwrap();
                } else {
                    println!("Error: Missing DeviceId argument for GetMotor command");
                }
            },
            LocalCommands::GetAnalog => {
                let mut device_id: Option<DeviceId> = None;
                for arg in parsed_command.args {
                    match arg {
                        ParsedArguments::DeviceId(id) => {
                            device_id = Some(id);
                        },
                        _ => {}
                    }
                }
                if let Some(id) = device_id {
                    println!("Requesting analog data from device {:?}", id);
                    let msg = CanDriveMessage::Control(
                        id,
                        Commands::GET_ANALOG,
                        ProtocolData::None
                    );
                    can_tx.send(msg).unwrap();
                } else {
                    println!("Error: Missing DeviceId argument for GetAnalog command");
                }
            },
            LocalCommands::GetServo => {
                let mut device_id: Option<DeviceId> = None;
                for arg in parsed_command.args {
                    match arg {
                        ParsedArguments::DeviceId(id) => {
                            device_id = Some(id);
                        },
                        _ => {}
                    }
                }
                if let Some(id) = device_id {
                    println!("Requesting servo data from device {:?}", id);
                    let msg = CanDriveMessage::Control(
                        id,
                        Commands::GET_SERVO,
                        ProtocolData::None
                    );
                    can_tx.send(msg).unwrap();
                } else {
                    println!("Error: Missing DeviceId argument for GetServo command");
                }
            },
            LocalCommands::SetServo0 => {
                // Example handling of SetServo0 command
                let mut device_id: Option<DeviceId> = None;
                let mut position: Option<u8> = None;
                for arg in parsed_command.args {
                    match arg {
                        ParsedArguments::DeviceId(id) => {
                            device_id = Some(id);
                        },
                        ParsedArguments::Position8(pos) => {
                            position = Some(pos);
                        },
                        _ => {}
                    }
                }
                if let (Some(id), Some(pos)) = (device_id, position) {
                    println!("Setting servo0 of device {:?} to {}", id, pos);
                    // Send command to device via comm_manager
                    let msg = CanDriveMessage::Control(
                        id,
                        Commands::SET_SERVO0,
                        ProtocolData::ServoPosition{value: pos}
                    );
                    can_tx.send(msg).unwrap();
                } else {
                    println!("Error: Missing arguments for SetServo0 command");
                }
            },
            LocalCommands::SetServo1 => {
                let mut device_id: Option<DeviceId> = None;
                let mut position: Option<u8> = None;
                for arg in parsed_command.args {
                    match arg {
                        ParsedArguments::DeviceId(id) => {
                            device_id = Some(id);
                        },
                        ParsedArguments::Position8(pos) => {
                            position = Some(pos);
                        },
                        _ => {}
                    }
                }
                if let (Some(id), Some(pos)) = (device_id, position) {
                    println!("Setting servo1 of device {:?} to {}", id, pos);
                    // Send command to device via comm_manager
                    let msg = CanDriveMessage::Control(
                        id,
                        Commands::SET_SERVO1,
                        ProtocolData::ServoPosition{value: pos}
                    );
                    can_tx.send(msg).unwrap();
                } else {
                    println!("Error: Missing arguments for SetServo1 command");
                }
            },
            LocalCommands::UpdateFirmware => {
                println!("Command not implemented yet");
            },
            _ => {
                println!("Command not implemented yet");
            }
        }
    }

    pub fn start(&self, cancel_token: CancellationToken, local_can_tx: &tokio::sync::mpsc::Sender<CanDriveMessage>, local_device_tx: &tokio::sync::mpsc::Sender<DeviceMessage>) -> Vec<tokio::JoinHandle<()>> {
        vec![tokio::spawn(async move {
            let mut buffer = String::new();
            let parser = LocalCommandParser::default();
            loop {
                tokio::select! {
                    _ = cancel_token.cancelled() => {
                        break;
                    },
                    status = io::stdin().read_line(&mut buffer) => {
                        if let Ok(status) = status {
                            if status == 0 {
                                break;
                            }
                            match parser.parse(buffer.clone()) {
                                Ok(cmd) => {
                                    // Handle command
                                parser.handle_command(cmd, cancel_token.clone(), &local_can_tx, &local_device_tx).await;
                                }
                                Err(e) => {
                                    eprintln!("Error parsing command: {}", e);
                                }
                            }
                            buffer.clear();
                        }
                    },
                };
            }
        })]
    }
}
