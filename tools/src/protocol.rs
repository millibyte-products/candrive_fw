use std::{collections::btree_map::Values, fmt::{Display, Formatter}, str::FromStr};

use byteorder::{ByteOrder, LittleEndian, WriteBytesExt, ReadBytesExt};

pub const CONTROLLER_ID: u16 = 0x00;
pub const DISCOVERY_ASSIGN_ID: u16 = 0x01;
pub const DISCOVERY_REQUEST_ID: u16 = 0x02;
pub const DEVICE_BASE_ID: u16 = 0x08;

#[repr(C)]
#[derive(Debug, Default, Clone, Copy)]
pub enum Commands {
    #[default]
    NOOP = 0x00,
    GET_INFO,
    SET_POSITION, // Fixed point, 16 bit
    GET_POSITION,         // Fixed point, 16 bit
    GET_STATUS,
    GET_ANALOG,
    SET_SERVO,
    GET_SERVO, // Device Servo values
    SET_LED,
    GET_LED,
    GET_MOTOR,
    SET_FOC,
    GET_FOC,
    // Streams allow uart <-> can bridging
    STREAM_START, // Start streaming to device UART
    STREAM_DATA,  // Stream write (TX) data
    ACK,
    START_FW_UPDATE,
    FW_UPDATE,
    NETWORK_RESET,
    ERROR,
    INVALID = 0x7F,
}

impl Commands {

    pub fn from_u8(value: u8) -> Commands {
        use Commands::*;
        match value {
            0 => NOOP,
            1 =>GET_INFO,
            2 =>SET_POSITION, // Fixed point, 16 bit
            3=>GET_POSITION,         // Fixed point, 16 bit
            4=>GET_STATUS,
            5=>GET_ANALOG,
            6=>SET_SERVO,
            7=>GET_SERVO, // Device Servo values
            8=>SET_LED,
            9=>GET_LED,
            10=>GET_MOTOR,
            11=>SET_FOC,
            12=>GET_FOC,
            // Streams allow uart <-> can bridging
            13=>STREAM_START, // Start streaming to device UART
            14=>STREAM_DATA,  // Stream write (TX) data
            15=>ACK,
            16=>START_FW_UPDATE,
            17=>FW_UPDATE,
            18=>NETWORK_RESET,
            19=>ERROR,
            _ => INVALID,
        }
    }

    pub fn to_u8(&self) -> u8 {
        return match self {
            Commands::NOOP => 0,
            Commands::GET_INFO => 1,
            Commands::SET_POSITION => 2,
            Commands::GET_POSITION => 3,
            Commands::GET_STATUS => 4,
            Commands::GET_ANALOG => 5,
            Commands::SET_SERVO => 6,
            Commands::GET_SERVO => 7,
            Commands::SET_LED => 8,
            Commands::GET_LED => 9,
            Commands::GET_MOTOR => 10,
            Commands::SET_FOC => 11,
            Commands::GET_FOC => 12,
            Commands::STREAM_START => 13,
            Commands::STREAM_DATA => 14,
            Commands::ACK => 15,
            Commands::START_FW_UPDATE => 16,
            Commands::FW_UPDATE => 17,
            Commands::NETWORK_RESET => 18,
            Commands::ERROR => 19,
            Commands::INVALID => 0x7F,
            _ => 0x7F,
        };
    }

    pub fn is_contoller_command(&self) -> bool {
        return self.to_u8() % 2 == 0;
    }

    pub fn is_device_command(&self) -> bool {
        return self.to_u8() % 2 == 1;
    }
}

#[derive(Debug, Default, Clone, Copy, Ord, Eq, PartialEq, PartialOrd, Hash)]
pub struct DeviceId {
    id: u8,
}

impl Display for DeviceId {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), std::fmt::Error> {
        write!(f, "{:?}", self.to_can_id());
        Ok(())
    }
}

impl FromStr for DeviceId {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let id = s.parse::<u8>().map_err(|e| e.to_string())?;
        return Ok(DeviceId::new(id));
    }
}

impl DeviceId {
    pub fn to_can_id(&self) -> u16 {
        return (self.id as u16) << 3;
    }

    pub fn from_can_id(id: u16) -> DeviceId {
        return DeviceId {
            id: (id >> 3) as u8,
        };
    }

    pub fn new(id: u8) -> DeviceId {
        return DeviceId { id };
    }

    pub fn to_u8(&self) -> u8 {
        return self.id;
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct StatusBits {
    endstop0: bool,
    endstop1: bool,
    misc: bool,
    fault: bool,
    mag: u8,
}

impl StatusBits {
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes: u8 = 0;
        if self.endstop0 {
            bytes |= 0x01;
        }
        if self.endstop1 {
            bytes |= 0x02;
        }
        if self.misc {
            bytes |= 0x04;
        }
        if self.fault {
            bytes |= 0x08;
        }
        bytes |= (self.mag & 0x0F) << 4;
        return vec![bytes];
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<StatusBits, &'static str> {
        if bytes.len() < 1 {
            return Err("Underflow parsing status struct");
        }
        let mut status = StatusBits {
            endstop0: false,
            endstop1: false,
            misc: false,
            fault: false,
            mag: 0,
        };
        status.endstop0 = (bytes[0] & 0x01) == 0x01;
        status.endstop1 = (bytes[0] & 0x02) == 0x02;
        status.misc = (bytes[0] & 0x04) == 0x04;
        status.fault = (bytes[0] & 0x08) == 0x08;
        status.mag = (bytes[0] & 0xF0) >> 4;
        return Ok(status);
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct MotorBits {
    value: u16,
    rst: bool,
    sleep: bool,
}

impl MotorBits {
    fn to_bytes(&self) -> Vec<u8> {
        let mut bytes: Vec<u8> = vec![0; 3];
        bytes[0] = (self.value & 0xFF) as u8;
        bytes[1] = ((self.value >> 8) & 0xFF) as u8;
        if self.rst {
            bytes[2] |= 0x01;
        }
        if self.sleep {
            bytes[2] |= 0x02;
        }
        return bytes;
    }

    fn from_bytes(bytes: &[u8]) -> Result<MotorBits, &'static str> {
        if bytes.len() < 3 {
            return Err("Underflow parsing motor struct");
        }
        let mut motor = MotorBits {
            value: 0,
            rst: false,
            sleep: false,
        };
        motor.value = ((bytes[0] as u16) | ((bytes[1] as u16) << 8)) & 0x0FFF;
        motor.rst = (bytes[2] & 0x01) == 0x01;
        motor.sleep = (bytes[2] & 0x02) == 0x02;
        return Ok(motor);
    }
}

#[repr(C)]
#[derive(Debug, Default, Clone, Copy)]
pub enum ProtocolData {
    #[default]
    Empty,
    Info {
        serial: u32,
        fw_version: u8,
        hw_version: u8,
    },
    Position {
        value: u16,
    },
    Status(StatusBits),
    Analog {
        a0: u16,
        a1: u16,
    },
    Servo {
        s0: u16,
        s1: u16,
        update_flag: u8,
    },
    Led {
        sys: u8,
        stat: u8,
        update_flag: u8,
    },
    Motor(MotorBits),
    FOC {
        foc_1: u8,
        foc_2: u8,
        foc_3: u8,
        en: u8,
    },
    StreamFragment {
        sequence: u8,
        data: [u8; 5],
    },
    Error {
        code: u8,
        message: u32,
    },
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub enum CanDriveMessage {
    DiscoveryReq {
        serial: u32,
        previous_id: u8,
    },
    DiscoveryAssign {
        serial: u32,
        assigned_id: u8,
    },
    Control {
        id: DeviceId,
        is_controller: bool,
        cmd: Commands,
        data: ProtocolData,
    },
}

impl CanDriveMessage {
    pub fn as_bytes(&self) -> Vec<u8> {
        let mut bytes: Vec<u8> = vec![];
        match self {
            CanDriveMessage::DiscoveryReq {
                serial,
                previous_id,
            } => {
                bytes.write_u32::<LittleEndian>(*serial);
                bytes.push(*previous_id);
            }
            CanDriveMessage::DiscoveryAssign {
                serial,
                assigned_id,
            } => {
                bytes.write_u32::<LittleEndian>(*serial);
                bytes.push(*assigned_id);
            }
            CanDriveMessage::Control { id:_, is_controller,cmd, data } => {
                let cmd_byte = cmd.to_u8() | (if *is_controller { 0x80 } else { 0x00 });
                bytes.push(cmd_byte);
                match data {
                    ProtocolData::Empty => {}
                    ProtocolData::Info {
                        serial,
                        fw_version,
                        hw_version,
                    } => {
                        bytes.write_u32::<LittleEndian>(*serial);
                        bytes.push(*fw_version);
                        bytes.push(*hw_version);
                    }
                    ProtocolData::Position { value } => {
                        bytes.write_u16::<LittleEndian>(*value);
                    }
                    ProtocolData::Status(status) => {
                        bytes.extend(status.to_bytes());
                    }
                    ProtocolData::Analog { a0, a1 } => {
                        bytes.write_u16::<LittleEndian>(*a0);
                        bytes.write_u16::<LittleEndian>(*a1);
                    }
                    ProtocolData::Servo { s0, s1 , update_flag} => {
                        bytes.write_u16::<LittleEndian>(*s0);
                        bytes.write_u16::<LittleEndian>(*s1);
                        bytes.push(*update_flag);
                    }
                    ProtocolData::Led{ sys, stat, update_flag} =>{
                        bytes.push(*sys);
                        bytes.push(*stat);
                        bytes.push(*update_flag);
                    }
                    ProtocolData::Motor(motor) => {
                        bytes.extend(motor.to_bytes());
                    }
                    ProtocolData::FOC {
                        foc_1,
                        foc_2,
                        foc_3,
                        en,
                    } => {
                        bytes.push(*foc_1);
                        bytes.push(*foc_2);
                        bytes.push(*foc_3);
                        bytes.push(*en);
                    }
                    ProtocolData::StreamFragment { sequence, data } => {
                        bytes.push(*sequence);
                        bytes.extend(data);
                    },
                    ProtocolData::Error { code, message } => {
                        bytes.push(*code);
                        bytes.write_u32::<LittleEndian>(*message);
                    }
                }
            }
        }
        return bytes;
    }

    pub fn parse_can_frame(id: u16, bytes: &[u8]) -> Result<Option<CanDriveMessage>, &'static str> {
        let cmd = bytes[0] & 0x7F;
        let is_contorller = (bytes[0] & 0x80) == 0x80;
        let data = &bytes[1..];
        let msg: CanDriveMessage;
        if id == CONTROLLER_ID {
            println!("Controller Message (ID: {:?})", bytes);
            return Ok(None);
        }
        if id == DISCOVERY_REQUEST_ID {
            print!("Discovery Req Message (ID: {:?})", bytes);
            msg = CanDriveMessage::DiscoveryReq {
                serial: LittleEndian::read_u32(&bytes[1..5]),
                previous_id: bytes[5],
            };
            return Ok(Some(msg));
        }
        if id == DISCOVERY_ASSIGN_ID {
            print!("Discovery Assign Message (ID: {:?})", bytes);
            msg = CanDriveMessage::DiscoveryAssign {
                serial: LittleEndian::read_u32(&bytes[1..5]),
                assigned_id: bytes[5],
            };
            return Ok(Some(msg));
        }
        if id < DEVICE_BASE_ID {
            println!("Reserved Message (ID: {:?})", bytes);
            return Ok(None);
        }
        let device_id = DeviceId::from_can_id(id);
        use Commands::*;
        msg = CanDriveMessage::Control {
            id: device_id,
            is_controller: is_contorller,
            cmd: Commands::from_u8(cmd),
            data: match Commands::from_u8(cmd) {
                GET_INFO => {
                    if data.len() < 6 {
                        return Err("Underflow device_info message");
                    }
                    ProtocolData::Info {
                        serial: LittleEndian::read_u32(&data[0..4]),
                        fw_version: data[4],
                        hw_version: data[5],
                    }
                }
                GET_POSITION => {
                    if data.len() < 2 {
                        return Err("Underflow device_position message");
                    }
                    ProtocolData::Position {
                        value: LittleEndian::read_u16(&data[0..2]),
                    }
                }
                GET_STATUS => {
                    if data.len() < 1 {
                        return Err("Underflow Device_status message");
                    }
                    ProtocolData::Status(StatusBits::from_bytes(&data[0..2])?)
                }
                GET_ANALOG => {
                    if data.len() < 4 {
                        return Err("Underflow device_analog message");
                    }
                    ProtocolData::Analog {
                        a0: LittleEndian::read_u16(&data[0..2]),
                        a1: LittleEndian::read_u16(&data[2..4]),
                    }
                }
                GET_SERVO => {
                    if data.len() < 4 {
                        return Err("Underflow device_servo message");
                    }
                    ProtocolData::Servo {
                        s0: LittleEndian::read_u16(&data[0..2]),
                        s1: LittleEndian::read_u16(&data[2..4]),
                        update_flag: data[4],
                    }
                }
                SET_LED => {
                    if data.len() < 3 {
                        return Err("Underflow device_set_led message");
                    }
                    ProtocolData::Led {
                        sys: data[0],
                        stat: data[1],
                        update_flag: data[2],
                    }
                }
                GET_LED => {
                    if data.len() < 3 {
                        return Err("Underflow device_led message");
                    }
                    ProtocolData::Led {
                        sys: data[0],
                        stat: data[1],
                        update_flag: data[2],
                    }
                }
                GET_MOTOR => {
                    if data.len() < 1 {
                        return Err("Underflow device_motor message");
                    }
                    ProtocolData::Motor(MotorBits::from_bytes(&data[0..])?)
                }
                GET_FOC => {
                    if data.len() < 4 {
                        return Err("Underflow device_foc message");
                    }
                    ProtocolData::FOC {
                        foc_1: data[0],
                        foc_2: data[1],
                        foc_3: data[2],
                        en: data[3],
                    }
                }
                STREAM_START => {
                    if data.len() < 5 {
                        return Err("Underflow device_stream_start message");
                    }
                    let mut s_data = [0; 5];
                    s_data.copy_from_slice(&data[1..6]);
                    ProtocolData::StreamFragment {
                        sequence: data[0],
                        data: s_data,
                    }
                }
                STREAM_DATA => {
                    if data.len() < 5 {
                        return Err("Underflow device_stream_data message");
                    }
                    let mut s_data = [0; 5];
                    s_data.copy_from_slice(&bytes[1..6]);
                    ProtocolData::StreamFragment {
                        sequence: data[0],
                        data: s_data,
                    }
                }
                ACK => ProtocolData::Empty,
                FW_UPDATE => ProtocolData::Empty,
                ERROR => {
                    if data.len() < 5 {
                        return Err("Underflow device_error message");
                    }
                    ProtocolData::Error {
                        code: data[0],
                        message: LittleEndian::read_u32(&data[1..5]),
                    }
                }
                _ => return Err("Invalid command"),
            },
        };
        Ok(Some(msg))
    }
}
