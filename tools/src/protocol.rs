use std::{fmt::{Display, Formatter, UpperHex}, str::FromStr};
use log::{debug};
use byteorder::{ByteOrder, LittleEndian, WriteBytesExt};
use strum_macros::{EnumDiscriminants, EnumIter};

pub const CONTROLLER_ID: u16 = 0x00;
pub const DISCOVERY_ASSIGN_ID: u16 = 0x01;
pub const DISCOVERY_REQUEST_ID: u16 = 0x02;
pub const DEVICE_BASE_ID: u16 = 0x08;
pub const MAX_DATA_LENGTH: usize = 8;

#[repr(u8)]
#[derive(Debug, Default, Clone, Copy, EnumDiscriminants)]
#[strum_discriminants(derive(EnumIter))]
#[strum_discriminants(name(CommandTypes))]
pub enum Commands {
    #[default]
    NOOP = 0x00,
    GetInfo,
    GetInfoExt,

    GetPosition, // Fixed point, 16 bit
    SetPosition,         // Fixed point, 16 bit

    GetStatus,

    GetAnalog,

    GetServo,
    SetServo, // Device Servo values

    GetLed,
    SetLed,

    GetMotor,
    SetMotor,

    GetFOC,
    SetFOC,
    // Streams allow uart <-> can bridging
    StreamStart, // Start streaming to device UART
    StreamData,  // Stream write (TX) data
    Ack,
    StartFwUpdate,
    FWUpdate,
    NetworkReset,
    OverwriteUserStore,
    EraseUserStore,
    RevokeConfig,
    Error,
    Invalid = 0x7F,
}

impl Commands {
    pub fn from_u8(value: u8) -> Commands {
        use Commands::*;
        match value
        {
            0 => NOOP,
            1 => GetInfo,
            2 => GetInfoExt,
            3 => GetPosition,         // Fixed point, 16 bit
            4 => SetPosition, // Fixed point, 16 bit
            5 => GetStatus,
            6 => GetAnalog,
            7 => GetServo,
            8 => SetServo,
            9 => GetLed,
            10 => SetLed,
            11 => GetMotor,
            12 => SetMotor,
            13 => GetFOC,
            14 => SetFOC,
            15 => StreamStart, // Start streaming to device UART
            16 => StreamData,  // Stream write (TX) data
            17 => Ack,
            18 => StartFwUpdate,
            19 => FWUpdate,
            20 => NetworkReset,
            21 => OverwriteUserStore,
            22 => EraseUserStore,
            23 => RevokeConfig,
            24 => Error,
            _ => Invalid,
        }
    }

    pub fn to_u8(&self) -> u8 {
        return self.index()
    }

    pub fn is_contoller_command(&self) -> bool {
        return self.to_u8() % 2 == 0;
    }

    pub fn is_device_command(&self) -> bool {
        return self.to_u8() % 2 == 1;
    }

    pub fn index(&self) -> u8 {
         // SAFETY: Because `Self` is marked `repr(u8)`, its layout is a `repr(C)` `union`
        // between `repr(C)` structs, each of which has the `u8` discriminant as its first
        // field, so we can read the discriminant without offsetting the pointer.
        unsafe { *<*const _>::from(self).cast::<u8>() }
    }
}

#[derive(Debug, Default, Clone, Copy, Ord, Eq, PartialEq, PartialOrd, Hash)]
pub struct DeviceId {
    id: u8,
}

impl UpperHex for DeviceId {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), std::fmt::Error> {
        write!(f, "{:X} ({:X})", self.to_u8(), self.to_can_id())
    }
}

impl Display for DeviceId {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), std::fmt::Error> {
        write!(f, "{:?} ({:X})", self.to_u8(), self.to_can_id())
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
        return (self.id as u16) + DEVICE_BASE_ID;
    }

    pub fn from_can_id(id: u16) -> DeviceId {
        return DeviceId {
            id: (id - DEVICE_BASE_ID) as u8,
        };
    }

    pub fn new(id: u8) -> DeviceId {
        return DeviceId { id };
    }

    pub fn to_u8(&self) -> u8 {
        return self.id;
    }

    pub fn from_u8(id: u8) -> DeviceId {
        return DeviceId { id };
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct StatusBits {
    pub endstop0: bool,
    pub endstop1: bool,
    pub misc: bool,
    pub fault: bool,
    pub mag: u8,
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
    pub value: u16,
    pub rst: bool,
    pub sleep: bool,
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
#[derive(Debug, Default, Clone, Copy, EnumDiscriminants)]
#[strum_discriminants(derive(EnumIter))]
#[strum_discriminants(name(ProtocolDataTypes))]
pub enum ProtocolData {
    #[default]
    Empty,
    Info {
        serial: u32,
        fw_major: u8,
        fw_minor: u8,
        fw_patch: u8,
    },
    InfoExt {
        flags: u8,
        temperature: u8,
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
    StreamStart {
        stream_target: u8,
        stream_length: u16,
        flags: u8,
    },
    StreamFragment {
        sequence: u8,
        data: [u8; 4],
    },
    RevokeConfig {
        device_id: u8,
    },
    Error {
        code: u8,
        message: u32,
    },
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, EnumDiscriminants)]
#[strum_discriminants(derive(EnumIter))]
#[strum_discriminants(name(CanDriveMessageTypes))]
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
    Broadcast {
        cmd: Commands,
        data: ProtocolData
    }
}

impl CanDriveMessage {
    pub fn as_bytes(&self) -> Result<Vec<u8>, String> {
        let mut bytes: Vec<u8> = vec![];
        fn handle_protocol_data(data: &ProtocolData) -> Result<Vec<u8>, String> {
            let mut bytes: Vec<u8> = vec![];
            match data {
                ProtocolData::Empty => {}
                ProtocolData::Info {
                    serial,
                    fw_major,
                    fw_minor,
                    fw_patch,
                } => {
                    bytes.write_u32::<LittleEndian>(*serial).map_err(|e| e.to_string())?;
                    bytes.push(*fw_major);
                    bytes.push(*fw_minor);
                    bytes.push(*fw_patch);
                }
                ProtocolData::InfoExt { flags, temperature }    => {
                    bytes.push(*flags);
                    bytes.write_u8(*temperature).map_err(|e| e.to_string())?;
                }
                ProtocolData::Position { value } => {
                    bytes.write_u16::<LittleEndian>(*value).map_err(|e|e.to_string())?;
                }
                ProtocolData::Status(status) => {
                    bytes.extend(status.to_bytes());
                }
                ProtocolData::Analog { a0, a1 } => {
                    bytes.write_u16::<LittleEndian>(*a0).map_err(|e| e.to_string())?;
                    bytes.write_u16::<LittleEndian>(*a1).map_err(|e| e.to_string())?;
                    bytes.write_u16::<LittleEndian>(*a1).map_err(|e| e.to_string())?;
                }
                ProtocolData::Servo { s0, s1 , update_flag} => {
                    bytes.write_u16::<LittleEndian>(*s0).map_err(|e| e.to_string())?;
                    bytes.write_u16::<LittleEndian>(*s1).map_err(|e| e.to_string())?;
                    bytes.write_u16::<LittleEndian>(*s1).map_err(|e|e.to_string())?;
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
                ProtocolData::StreamStart { stream_target, stream_length, flags } => {
                    bytes.push(*stream_target);
                    bytes.write_u16::<LittleEndian>(*stream_length).map_err(|e| e.to_string())?;
                    bytes.push(*flags);
                }
                ProtocolData::StreamFragment { sequence, data } => {
                    bytes.push(*sequence);
                    bytes.extend(data);
                },
                ProtocolData::RevokeConfig { device_id } => {
                    bytes.push(*device_id);
                },
                ProtocolData::Error { code, message } => {
                    bytes.push(*code);
                    bytes.write_u32::<LittleEndian>(*message).map_err(|e| e.to_string())?;
                }
            }
            Ok(bytes)
        }
        match self {
            CanDriveMessage::DiscoveryReq {
                serial,
                previous_id,
            } => {
                bytes.write_u32::<LittleEndian>(*serial).map_err(|e| e.to_string())?;
                bytes.push(*previous_id);
            }
            CanDriveMessage::DiscoveryAssign {
                serial,
                assigned_id,
            } => {
                bytes.write_u32::<LittleEndian>(*serial).map_err(|e| e.to_string())?;
                bytes.push(*assigned_id);
            }
            CanDriveMessage::Control { id:_, is_controller,cmd, data } => {
                let cmd_byte = cmd.to_u8() | (if *is_controller { 0x80 } else { 0x00 });
                bytes.push(cmd_byte);
                bytes.extend(handle_protocol_data(data)?);
            }
            CanDriveMessage::Broadcast { cmd, data } => {
                bytes.push(cmd.to_u8() | 0x80);
                bytes.extend(handle_protocol_data(data)?);
            }
        }
        return Ok(bytes);
    }

    pub fn parse_can_frame(id: u16, bytes: &[u8]) -> Result<Option<CanDriveMessage>, String> {
        let cmd = bytes[0] & 0x7F;
        let is_controller = (bytes[0] & 0x80) == 0x80;
        let data = &bytes[1..];
        let msg: CanDriveMessage;
        if id == CONTROLLER_ID {
            debug!("Controller Message (ID: {:?})", bytes);
            return Ok(None);
        }
        if id == DISCOVERY_REQUEST_ID {
            if bytes.len() < 5 {
                return Err("Underflow discovery request message".to_string());
            }
            debug!("Discovery Request Message ({:?})", bytes);
            msg = CanDriveMessage::DiscoveryReq {
                serial: LittleEndian::read_u32(&bytes[0..4]),
                previous_id: bytes[4],
            };
            return Ok(Some(msg));
        }
        if id == DISCOVERY_ASSIGN_ID {
            if bytes.len() < 5 {
                return Err("Underflow discovery assign message".to_string());
            }
            debug!("Discovery Assign Message ({:?})", bytes);
            msg = CanDriveMessage::DiscoveryAssign {
                serial: LittleEndian::read_u32(&bytes[0..4]),
                assigned_id: bytes[4],
            };
            return Ok(Some(msg));
        }
        if id < DEVICE_BASE_ID {
            debug!("Reserved Message ({:?})", bytes);
            return Ok(None);
        }
        debug!("Control Message ({:?})", bytes);
        let device_id = DeviceId::from_can_id(id);
        use Commands::*;
        msg = CanDriveMessage::Control {
            id: device_id,
            is_controller: is_controller,
            cmd: Commands::from_u8(cmd),
            data: match Commands::from_u8(cmd) {
                NOOP => ProtocolData::Empty,
                GetInfo => {
                    if data.len() < 6 {
                        return Err("Underflow device_info message".to_string());
                    }
                    ProtocolData::Info {
                        serial: LittleEndian::read_u32(&data[0..4]),
                        fw_major: data[4],
                        fw_minor: data[5],
                        fw_patch: data[6],
                    }
                }
                GetInfoExt => {
                    if data.len() < 2 {
                        return Err("Underflow device_info_ext message".to_string());
                    }
                    ProtocolData::InfoExt {
                        flags: data[0],
                        temperature: data[1],
                    }
                }
                GetPosition => {
                    if data.len() < 2 {
                        return Err("Underflow device_position message".to_string());
                    }
                    ProtocolData::Position {
                        value: LittleEndian::read_u16(&data[0..2]),
                    }
                }
                SetPosition => {
                    if data.len() < 2 {
                        return Err("Underflow device_set_position message".to_string());
                    }
                    ProtocolData::Position {
                        value: LittleEndian::read_u16(&data[0..2]),
                    }
                }
                GetStatus => {
                    if data.len() < 1 {
                        return Err("Underflow Device_status message".to_string());
                    }
                    ProtocolData::Status(StatusBits::from_bytes(&data[0..1])?)
                }
                GetAnalog => {
                    if data.len() < 4 {
                        return Err("Underflow device_analog message".to_string());
                    }
                    ProtocolData::Analog {
                        a0: LittleEndian::read_u16(&data[0..2]),
                        a1: LittleEndian::read_u16(&data[2..4]),
                    }
                }
                SetServo => {ProtocolData::Empty},
                GetServo => {
                    if data.len() < 4 {
                        return Err("Underflow device_servo message".to_string());
                    }
                    ProtocolData::Servo {
                        s0: LittleEndian::read_u16(&data[0..2]),
                        s1: LittleEndian::read_u16(&data[2..4]),
                        update_flag: data[4],
                    }
                }
                SetLed => {
                    if data.len() < 3 {
                        return Err("Underflow device_set_led message".to_string());
                    }
                    ProtocolData::Led {
                        sys: data[0],
                        stat: data[1],
                        update_flag: data[2],
                    }
                }
                GetLed => {
                    if data.len() < 3 {
                        return Err("Underflow device_led message".to_string());
                    }
                    ProtocolData::Led {
                        sys: data[0],
                        stat: data[1],
                        update_flag: data[2],
                    }
                }
                GetMotor => {
                    if data.len() < 3 {
                        return Err("Underflow device_motor message".to_string());
                    }
                    ProtocolData::Motor(MotorBits::from_bytes(&data[0..])?)
                }
                SetMotor => {
                    if data.len() < 3 {
                        return Err("Underflow device_motor message".to_string());
                    }
                    ProtocolData::Motor(MotorBits::from_bytes(&data[0..])?)
                }
                GetFOC => {
                    if data.len() < 4 {
                        return Err("Underflow device_foc message".to_string());
                    }
                    ProtocolData::FOC {
                        foc_1: data[0],
                        foc_2: data[1],
                        foc_3: data[2],
                        en: data[3],
                    }
                }
                SetFOC => {
                    if data.len() < 4 {
                        return Err("Underflow device_foc message".to_string());
                    }
                    ProtocolData::FOC {
                        foc_1: data[0],
                        foc_2: data[1],
                        foc_3: data[2],
                        en: data[3],
                    }
                }
                StreamStart => {
                    if data.len() < 4 {
                        return Err("Underflow device_stream_start message".to_string());
                    }
                    ProtocolData::StreamStart {
                        stream_target: data[0],
                        stream_length: LittleEndian::read_u16(&data[1..3]),
                        flags: data[3],
                    }
                }
                StreamData => {
                    if data.len() < 4 {
                        return Err("Underflow device_stream_data message".to_string());
                    }
                    let mut s_data = [0; 4];
                    s_data.copy_from_slice(&bytes[1..5]);
                    ProtocolData::StreamFragment {
                        sequence: data[0],
                        data: s_data,
                    }
                }
                Ack => ProtocolData::Empty,
                StartFwUpdate => ProtocolData::Empty,
                FWUpdate => ProtocolData::Empty,
                RevokeConfig => ProtocolData::RevokeConfig {
                    device_id: data[0],
                },
                NetworkReset => ProtocolData::Empty,
                OverwriteUserStore => ProtocolData::Empty,
                EraseUserStore => ProtocolData::Empty,
                Error => {
                    if data.len() < 5 {
                        return Err("Underflow device_error message".to_string());
                    }
                    ProtocolData::Error {
                        code: data[0],
                        message: LittleEndian::read_u32(&data[1..5]),
                    }
                }
                Invalid => {
                    return Err("Invalid command in CAN frame".to_string());
                }
            },
        };
        Ok(Some(msg))
    }

    pub fn index(&self) -> u8 {
         // SAFETY: Because `Self` is marked `repr(u8)`, its layout is a `repr(C)` `union`
        // between `repr(C)` structs, each of which has the `u8` discriminant as its first
        // field, so we can read the discriminant without offsetting the pointer.
        unsafe { *<*const _>::from(self).cast::<u8>() }
    }
}
