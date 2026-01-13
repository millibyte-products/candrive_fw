use std::collections::btree_map::Values;

use byteorder::{ByteOrder, LittleEndian};

pub const CONTROLLER_ID: u16 = 0x00;
pub const DISCOVERY_ASSIGN_ID: u16 = 0x01;
pub const DISCOVERY_REQUEST_ID: u16 = 0x02;
pub const DEVICE_BASE_ID: u16 = 0x08;

#[repr(C)]
pub enum Commands {
    CONTROLLER_GET_INFO = 0,
    DEVICE_INFO,
    CONTROLLER_SET_POSITION, // Fixed point, 16 bit
    DEVICE_POSITION,         // Fixed point, 16 bit
    CONTROLLER_GET_STATUS,
    DEVICE_STATUS, // Device status, position, motor fault, endstops
    CONTROLLER_GET_ANALOG,
    DEVICE_ANALOG, // Device ADC reads (2 x 16 bit)
    CONTROLLER_SET_SERVO,
    DEVICE_SERVO, // Device Servo values
    CONTROLLER_SET_LED,
    DEVICE_LED, // Device led values
    CONTROLLER_SET_MOTOR,
    DEVICE_MOTOR,
    CONTROLLER_SET_FOC,
    DEVICE_FOC,
    // Streams allow uart <-> can bridging
    CONTROLLER_STREAM_START, // Start streaming to device UART
    DEVICE_STREAM_START,     // Start streaming from device UART
    CONTROLLER_STREAM_DATA,  // Stream write (TX) data
    DEVICE_STREAM_DATA,      // Stream read (RX) data
    CONTROLLER_ACK,
    DEVICE_ACK,
    CONTROLLER_START_FW_UPDATE,
    DEVICE_FW_UPDATE,
}

impl Commands {
    pub fn from_u8(value: u8) -> Result<Commands, &'static str> {
        return Ok(match value {
            0 => Commands::CONTROLLER_GET_INFO,
            1 => Commands::DEVICE_INFO,
            2 => Commands::CONTROLLER_SET_POSITION,
            3 => Commands::DEVICE_POSITION,
            4 => Commands::CONTROLLER_GET_STATUS,
            5 => Commands::DEVICE_STATUS,
            6 => Commands::CONTROLLER_GET_ANALOG,
            7 => Commands::DEVICE_ANALOG,
            8 => Commands::CONTROLLER_SET_SERVO,
            9 => Commands::DEVICE_SERVO,
            10 => Commands::CONTROLLER_SET_LED,
            11 => Commands::DEVICE_LED,
            12 => Commands::CONTROLLER_SET_MOTOR,
            13 => Commands::DEVICE_MOTOR,
            14 => Commands::CONTROLLER_SET_FOC,
            15 => Commands::DEVICE_FOC,
            16 => Commands::CONTROLLER_STREAM_START,
            17 => Commands::DEVICE_STREAM_START,
            18 => Commands::CONTROLLER_STREAM_DATA,
            19 => Commands::DEVICE_STREAM_DATA,
            20 => Commands::CONTROLLER_ACK,
            21 => Commands::DEVICE_ACK,
            22 => Commands::CONTROLLER_START_FW_UPDATE,
            23 => Commands::DEVICE_FW_UPDATE,
            _ => {
                return Err("Invalid command");
            }
        });
    }

    pub fn to_u8(&self) -> u8 {
        return match self {
            Commands::CONTROLLER_GET_INFO => 0,
            Commands::DEVICE_INFO => 1,
            Commands::CONTROLLER_SET_POSITION => 2,
            Commands::DEVICE_POSITION => 3,
            Commands::CONTROLLER_GET_STATUS => 4,
            Commands::DEVICE_STATUS => 5,
            Commands::CONTROLLER_GET_ANALOG => 6,
            Commands::DEVICE_ANALOG => 7,
            Commands::CONTROLLER_SET_SERVO => 8,
            Commands::DEVICE_SERVO => 9,
            Commands::CONTROLLER_SET_LED => 10,
            Commands::DEVICE_LED => 11,
            Commands::CONTROLLER_SET_MOTOR => 12,
            Commands::DEVICE_MOTOR => 13,
            Commands::CONTROLLER_SET_FOC => 14,
            Commands::DEVICE_FOC => 15,
            Commands::CONTROLLER_STREAM_START => 16,
            Commands::DEVICE_STREAM_START => 17,
            Commands::CONTROLLER_STREAM_DATA => 18,
            Commands::DEVICE_STREAM_DATA => 19,
            Commands::CONTROLLER_ACK => 20,
            Commands::DEVICE_ACK => 21,
            Commands::CONTROLLER_START_FW_UPDATE => 22,
            Commands::DEVICE_FW_UPDATE => 23,
        };
    }

    pub fn is_contoller_command(&self) -> bool {
        return self.to_u8() % 2 == 0;
    }

    pub fn is_device_command(&self) -> bool {
        return self.to_u8() % 2 == 1;
    }
}

#[derive(Debug, Default, Clone, Copy)]
struct DeviceId {
    id: u8,
}

impl DeviceId {
    fn to_can_id(&self) -> u16 {
        return (self.id as u16) << 3;
    }

    fn from_can_id(id: u16) -> DeviceId {
        return DeviceId {
            id: (id >> 3) as u8,
        };
    }

    fn new(id: u8) -> DeviceId {
        return DeviceId { id };
    }

    fn to_u8(&self) -> u8 {
        return self.id;
    }
}

struct StatusBits {
    endstop0: bool,
    endstop1: bool,
    misc: bool,
    fault: bool,
    mag: u8,
}

impl StatusBits {
    fn to_bytes(&self) -> Vec<u8> {
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

    fn from_bytes(bytes: &[u8]) -> Result<StatusBits, &'static str> {
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

struct MotorBits {
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

struct LedBits {
    stat: bool,
    sys: bool,
}

impl LedBits {
    fn to_bytes(&self) -> Vec<u8> {
        let mut bytes: Vec<u8> = vec![0; 2];
        if self.stat {
            bytes[0] |= 0x01;
        }
        if self.sys {
            bytes[0] |= 0x02;
        }
        return bytes;
    }

    fn from_bytes(bytes: &[u8]) -> Result<LedBits, &'static str> {
        if bytes.len() < 2 {
            return Err("Underflow parsing led struct");
        }
        let mut led = LedBits {
            stat: false,
            sys: false,
        };
        led.stat = (bytes[0] & 0x01) == 0x01;
        led.sys = (bytes[0] & 0x02) == 0x02;
        return Ok(led);
    }
}

#[repr(C)]
enum ProtocolData {
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
    },
    Led(LedBits),
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
}

#[repr(C)]
enum CanDriveMessage {
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
        cmd: Commands,
        data: ProtocolData,
    },
}

impl CanDriveMessage {
    fn as_bytes(&self) -> Vec<u8> {
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
                bytes.write_u32::<LittleEndian>(&mut bytes[0..4], *serial);
                bytes.push(*assigned_id);
            }
            CanDriveMessage::Control { id, cmd, data } => {
                bytes.push(cmd.to_u8());
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
                        butes.write_u16::<LittleEndian>(*value);
                    }
                    ProtocolData::Status(status) => {
                        bytes.extend(status.to_bytes());
                    }
                    ProtocolData::Analog { a0, a1 } => {
                        bytes.write_u16::<LittleEndian>(*a0);
                        bytes.write_u16::<LittleEndian>(*a1);
                    }
                    ProtocolData::Servo { s0, s1 } => {
                        bytes.write_u16::<LittleEndian>(*s0);
                        bytes.write_u16::<LittleEndian>(*s1);
                    }
                    ProtocolData::Led(led) => {
                        bytes.extend(led.to_bytes());
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
                    }
                }
            }
        }
        return bytes;
    }

    fn parse_can_frame(id: u16, bytes: &[u8]) -> Result<Option<CanDriveMessage>, &'static str> {
        let cmd = bytes[0];
        let data = &bytes[1..];
        let mut msg: CanDriveMessage;
        if id == CONTROLLER_ID {
            println!("Controller Message (ID: {:?})", bytes);
            return Ok(None);
        }
        if id == DISCOVERY_REQUEST_ID {
            print!("Discovery Message (ID: {:?})", bytes);
            msg = CanDriveMessage::Discovery {
                serial: LittleEndian::read_u32(&bytes[1..5]),
                previoud_id: bytes[5],
            };
            return Ok(Some(msg));
        }
        if id == DISCOVERY_ASSIGN_ID {
            print!("Discovery Message (ID: {:?})", bytes);
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
        msg = CanDriveMessage::Control {
            id: device_id,
            cmd: cmd.to_u8(),
            data: match cmd {
                DEVICE_INFO => {
                    if data.len() < 6 {
                        return Err("Underflow device_info message");
                    }
                    ProtocolData::Info {
                        serial: LittleEndian::read_u32(&data[0..4]),
                        fw_version: data[4],
                        hw_version: data[5],
                    }
                }
                DEVICE_POSITION => {
                    if data.len() < 2 {
                        return Err("Underflow device_position message");
                    }
                    ProtocolData::Position {
                        value: LittleEndian::read_u16(&data[0..2]),
                    }
                }
                DEVICE_STATUS => {
                    if data.len() < 1 {
                        return Err("Underflow Device_status message");
                    }
                    ProtocolData::Status(StatusBits::from_bytes(&data[0..2])?)
                }
                DEVICE_ANALOG => {
                    if data.len() < 4 {
                        return Err("Underflow device_analog message");
                    }
                    ProtocolData::Analog {
                        a0: LittleEndian::read_u16(&data[0..2]),
                        a1: LittleEndian::read_u16(&data[2..4]),
                    }
                }
                DEVICE_SERVO => {
                    if data.len() < 4 {
                        return Err("Underflow device_servo message");
                    }
                    ProtocolData::Servo {
                        s0: LittleEndian::read_u16(&data[0..2]),
                        s1: LittleEndian::read_u16(&data[2..4]),
                    }
                }
                DEVICE_LED => {
                    if data.len() < 1 {
                        return Err("Underflow device_led message");
                    }
                    ProtocolData::Led(LedBits::from_bytes(&data[0..1])?)
                }
                DEVICE_MOTOR => {
                    if data.len() < 1 {
                        return Err("Underflow device_motor message");
                    }
                    ProtocolData::Motor(MotorBits::from_bytes(&data[0..])?)
                }
                DEVICE_FOC => {
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
                DEVICE_STREAM_START => {
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
                DEVICE_STREAM_DATA => {
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
                DEVICE_ACK => ProtocolData::Empty,
                DEVICE_FW_UPDATE => ProtocolData::Empty,
                _ => return Err("Invalid command"),
            },
        };
        Ok(Some(msg))
    }
}
