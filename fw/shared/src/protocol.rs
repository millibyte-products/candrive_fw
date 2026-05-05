//! candrive on-the-wire protocol — shared between firmware and host.
//!
//! This module is the source of truth for command IDs and payload encoding.
//! The host's `tools/src/protocol.rs` will eventually be replaced by a
//! thin std-friendly wrapper around this module; for now both must stay
//! byte-identical (encoding + command numbering).
//!
//! Constraints (because this lands in the no-std common image):
//!   - No `alloc`, no external deps.
//!   - All buffers are `[u8; 8]` (one classic-CAN frame). Encoders take
//!     a mutable slice and return the number of bytes written; decoders
//!     take `&[u8]`.

use crate::can_frame::{CanFrame, CAN_MAX_DLEN};

// ---- CAN-ID address space -------------------------------------------------

pub const CONTROLLER_ID: u16        = 0x00;
pub const DISCOVERY_ASSIGN_ID: u16  = 0x01;
pub const DISCOVERY_REQUEST_ID: u16 = 0x02;
pub const DEVICE_BASE_ID: u16       = 0x08;

/// Top bit of the command byte = "this is a controller→device message".
pub const CONTROLLER_BIT: u8 = 0x80;
pub const COMMAND_MASK:   u8 = 0x7F;

// ---- Commands -------------------------------------------------------------

/// Kept numerically identical to `tools/src/protocol.rs` `Commands`.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    Noop            = 0x00,
    GetInfo         = 0x01,
    GetInfoExt      = 0x02,
    GetPosition     = 0x03,
    SetPosition     = 0x04,
    GetStatus       = 0x05,
    GetAnalog       = 0x06,
    GetServo        = 0x07,
    SetServo        = 0x08,
    GetLed          = 0x09,
    SetLed          = 0x0A,
    GetMotor        = 0x0B,
    SetMotor        = 0x0C,
    GetFoc          = 0x0D,
    SetFoc          = 0x0E,
    StreamStart     = 0x0F,
    StreamRead      = 0x10,
    StreamWrite     = 0x11,
    Ack             = 0x12,
    FirmwareUpdate  = 0x13,
    UserStoreUpdate = 0x14,
    EraseUserStore  = 0x15,
    NetworkReset    = 0x16,
    RevokeConfig    = 0x17,
    Error           = 0x18,
    StreamCommit    = 0x19,
    GetMotorParam   = 0x20,
    SetMotorParam   = 0x21,
    RunCalibration  = 0x22,
    SetMotorCommand = 0x23,
    SaveMotorParams = 0x24,
    Invalid         = 0x7F,
}

impl Command {
    pub fn from_u8(v: u8) -> Self {
        use Command::*;
        match v & COMMAND_MASK {
            0x00 => Noop,
            0x01 => GetInfo,
            0x02 => GetInfoExt,
            0x03 => GetPosition,
            0x04 => SetPosition,
            0x05 => GetStatus,
            0x06 => GetAnalog,
            0x07 => GetServo,
            0x08 => SetServo,
            0x09 => GetLed,
            0x0A => SetLed,
            0x0B => GetMotor,
            0x0C => SetMotor,
            0x0D => GetFoc,
            0x0E => SetFoc,
            0x0F => StreamStart,
            0x10 => StreamRead,
            0x11 => StreamWrite,
            0x12 => Ack,
            0x13 => FirmwareUpdate,
            0x14 => UserStoreUpdate,
            0x15 => EraseUserStore,
            0x16 => NetworkReset,
            0x17 => RevokeConfig,
            0x18 => Error,
            0x19 => StreamCommit,
            0x20 => GetMotorParam,
            0x21 => SetMotorParam,
            0x22 => RunCalibration,
            0x23 => SetMotorCommand,
            0x24 => SaveMotorParams,
            _    => Invalid,
        }
    }

    #[inline]
    pub fn to_u8(self) -> u8 { self as u8 }
}

// ---- Status / Motor sub-encodings ----------------------------------------

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StatusBits {
    pub endstop0: bool,
    pub endstop1: bool,
    pub misc:     bool,
    pub fault:    bool,
    pub mag:      u8,   // 0..=15
}

impl StatusBits {
    pub fn to_byte(&self) -> u8 {
        let mut b = 0u8;
        if self.endstop0 { b |= 0x01; }
        if self.endstop1 { b |= 0x02; }
        if self.misc     { b |= 0x04; }
        if self.fault    { b |= 0x08; }
        b |= (self.mag & 0x0F) << 4;
        b
    }
    pub fn from_byte(b: u8) -> Self {
        Self {
            endstop0: b & 0x01 != 0,
            endstop1: b & 0x02 != 0,
            misc:     b & 0x04 != 0,
            fault:    b & 0x08 != 0,
            mag:      (b >> 4) & 0x0F,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MotorBits {
    pub value: u16,   // 12-bit duty
    pub rst:   bool,
    pub sleep: bool,
}

impl MotorBits {
    pub fn to_bytes(&self) -> [u8; 3] {
        let mut b = [0u8; 3];
        b[0] = self.value as u8;
        b[1] = (self.value >> 8) as u8;
        if self.rst   { b[2] |= 0x01; }
        if self.sleep { b[2] |= 0x02; }
        b
    }
    pub fn from_bytes(b: &[u8]) -> Result<Self, DecodeError> {
        if b.len() < 3 { return Err(DecodeError::Truncated); }
        Ok(Self {
            value: ((b[0] as u16) | ((b[1] as u16) << 8)) & 0x0FFF,
            rst:   b[2] & 0x01 != 0,
            sleep: b[2] & 0x02 != 0,
        })
    }
}

// ---- Payloads -------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ProtocolData {
    Empty,
    Info        { serial: u32, fw_major: u8, fw_minor: u8, fw_patch: u8 },
    InfoExt     { flags: u8, temperature: u8 },
    /// Single-turn mechanical angle in **Q-format radians**: `value`
    /// represents `angle_rad * 65536 / (2π)`, covering `[0, 2π)`.
    /// 1 LSB ≈ 95.9 μrad. The 14-bit MT6701 encoder maps exactly via
    /// `value = counts << 2`, so the wire form preserves full encoder
    /// resolution with 4 bits of headroom for any future upgrade.
    Position    { value: u16 },
    Status(StatusBits),
    Analog      { a0: u16, a1: u16 },
    Servo       { s0: u16, s1: u16, update_flag: u8 },
    Led         { sys: u8, stat: u8, update_flag: u8 },
    Motor(MotorBits),
    Foc         { foc_1: u8, foc_2: u8, foc_3: u8, en: u8 },
    /// `stream_length` only — checksum / CRC verification is out-of-band
    /// (firmware update is followed by a `GetInfo` round-trip from the host
    /// to confirm the new image actually boots).
    StreamStart { stream_length: u32 },
    StreamFragment([u8; 7]),
    /// CRC-32/MPEG-2 over the streamed bytes (matches `crc32_mpeg2`).
    /// Sent by the host after the last `StreamWrite`; the bootloader
    /// re-computes locally and Acks only on match.
    StreamCommit { crc32: u32 },
    RevokeConfig { serial_no: u32 },
    Error       { code: u8, message: u32 },
    /// `index` selects which scalar parameter (see app/src/motor.rs for the
    /// authoritative table). `value` is little-endian IEEE-754 f32 to keep
    /// the wire form simple and exchange-friendly.
    MotorParam  { index: u8, value: f32 },
    /// Host → device: kick off a pole-pair / zero-offset calibration
    /// run. `freq_dhz` is the electrical sweep frequency in tenths of a
    /// Hz (so 10 == 1 Hz). `dur_ms` is the sweep duration. `valign_pct`
    /// is the alignment voltage as a percentage of `voltage_supply`
    /// (capped at 100). 5 bytes on the wire.
    CalibrationReq    { freq_dhz: u16, dur_ms: u16, valign_pct: u8 },
    /// Device → host: result of a calibration run. `flags` bit 0 = fault
    /// asserted during run. 7 bytes on the wire.
    CalibrationResult { pole_pairs: f32, zero_offset: u16, flags: u8 },
    /// Host → device: set the closed-loop control mode and setpoint.
    /// Modes:
    ///   0 = Idle (bridge disabled)
    ///   1 = Voltage  (target = Vq volts, signed)
    ///   2 = Velocity (target = mech rad/s)
    ///   3 = Position (target = absolute multi-turn mech radians)
    ///   4 = OpenLoop (target = elec rad/s, diagnostic)
    ///   5 = PositionAbsShortest (target wrapped to (−π,π], shortest path)
    ///   6 = PositionAbsForward  (target wrapped to (−π,π], +ve rotation)
    ///   7 = PositionAbsBackward (target wrapped to (−π,π], −ve rotation)
    ///   8 = PositionRelative    (target = signed delta from current angle)
    /// 5 bytes on the wire.
    MotorCommand { mode: u8, target: f32 },
}

// ---- Top-level framing ----------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Message {
    /// CAN ID 0x02
    DiscoveryReq    { serial: u32, previous_id: u8 },
    /// CAN ID 0x01
    DiscoveryAssign { serial: u32, assigned_id: u8 },
    /// CAN ID >= 0x08; per-device addressed traffic.
    Control {
        device_id:     u8,
        is_controller: bool,
        cmd:           Command,
        data:          ProtocolData,
    },
    /// CAN ID 0x00 — not currently parsed inbound; we surface raw to the
    /// application so it can implement controller-specific behaviour.
    Controller([u8; CAN_MAX_DLEN], u8 /* len */),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    Empty,
    Truncated,
    InvalidCommand,
    UnknownAddress,
}

// ---- Encode helpers -------------------------------------------------------

#[inline] fn put_u16(b: &mut [u8], v: u16) { b[0] = v as u8; b[1] = (v >> 8) as u8; }
#[inline] fn put_u32(b: &mut [u8], v: u32) {
    b[0] = v as u8;
    b[1] = (v >>  8) as u8;
    b[2] = (v >> 16) as u8;
    b[3] = (v >> 24) as u8;
}
#[inline] fn get_u16(b: &[u8]) -> u16 { (b[0] as u16) | ((b[1] as u16) << 8) }
#[inline] fn get_u32(b: &[u8]) -> u32 {
    (b[0] as u32)
        | ((b[1] as u32) << 8)
        | ((b[2] as u32) << 16)
        | ((b[3] as u32) << 24)
}

fn encode_data(data: &ProtocolData, out: &mut [u8]) -> Result<usize, DecodeError> {
    use ProtocolData::*;
    match data {
        Empty => Ok(0),
        Info { serial, fw_major, fw_minor, fw_patch } => {
            if out.len() < 7 { return Err(DecodeError::Truncated); }
            put_u32(&mut out[0..4], *serial);
            out[4] = *fw_major; out[5] = *fw_minor; out[6] = *fw_patch;
            Ok(7)
        }
        InfoExt { flags, temperature } => {
            if out.len() < 2 { return Err(DecodeError::Truncated); }
            out[0] = *flags; out[1] = *temperature;
            Ok(2)
        }
        Position { value } => {
            if out.len() < 2 { return Err(DecodeError::Truncated); }
            put_u16(&mut out[0..2], *value);
            Ok(2)
        }
        Status(s) => {
            if out.is_empty() { return Err(DecodeError::Truncated); }
            out[0] = s.to_byte();
            Ok(1)
        }
        Analog { a0, a1 } => {
            // NB: legacy host writes a1 twice (4 bytes total but two copies of a1).
            // We follow the host spec literally so the wire matches.
            if out.len() < 6 { return Err(DecodeError::Truncated); }
            put_u16(&mut out[0..2], *a0);
            put_u16(&mut out[2..4], *a1);
            put_u16(&mut out[4..6], *a1);
            Ok(6)
        }
        Servo { s0, s1, update_flag } => {
            if out.len() < 7 { return Err(DecodeError::Truncated); }
            put_u16(&mut out[0..2], *s0);
            put_u16(&mut out[2..4], *s1);
            put_u16(&mut out[4..6], *s1);
            out[6] = *update_flag;
            Ok(7)
        }
        Led { sys, stat, update_flag } => {
            if out.len() < 3 { return Err(DecodeError::Truncated); }
            out[0] = *sys; out[1] = *stat; out[2] = *update_flag;
            Ok(3)
        }
        Motor(m) => {
            if out.len() < 3 { return Err(DecodeError::Truncated); }
            let b = m.to_bytes();
            out[..3].copy_from_slice(&b);
            Ok(3)
        }
        Foc { foc_1, foc_2, foc_3, en } => {
            if out.len() < 4 { return Err(DecodeError::Truncated); }
            out[0] = *foc_1; out[1] = *foc_2; out[2] = *foc_3; out[3] = *en;
            Ok(4)
        }
        StreamStart { stream_length } => {
            if out.len() < 4 { return Err(DecodeError::Truncated); }
            put_u32(&mut out[0..4], *stream_length);
            Ok(4)
        }
        StreamFragment(d) => {
            if out.len() < 7 { return Err(DecodeError::Truncated); }
            out[..7].copy_from_slice(d);
            Ok(7)
        }
        StreamCommit { crc32 } => {
            if out.len() < 4 { return Err(DecodeError::Truncated); }
            put_u32(&mut out[0..4], *crc32);
            Ok(4)
        }
        RevokeConfig { serial_no } => {
            if out.len() < 4 { return Err(DecodeError::Truncated); }
            put_u32(&mut out[0..4], *serial_no);
            Ok(4)
        }
        Error { code, message } => {
            if out.len() < 5 { return Err(DecodeError::Truncated); }
            out[0] = *code;
            put_u32(&mut out[1..5], *message);
            Ok(5)
        }
        MotorParam { index, value } => {
            if out.len() < 5 { return Err(DecodeError::Truncated); }
            out[0] = *index;
            put_u32(&mut out[1..5], value.to_bits());
            Ok(5)
        }
        CalibrationReq { freq_dhz, dur_ms, valign_pct } => {
            if out.len() < 5 { return Err(DecodeError::Truncated); }
            put_u16(&mut out[0..2], *freq_dhz);
            put_u16(&mut out[2..4], *dur_ms);
            out[4] = *valign_pct;
            Ok(5)
        }
        CalibrationResult { pole_pairs, zero_offset, flags } => {
            if out.len() < 7 { return Err(DecodeError::Truncated); }
            put_u32(&mut out[0..4], pole_pairs.to_bits());
            put_u16(&mut out[4..6], *zero_offset);
            out[6] = *flags;
            Ok(7)
        }
        MotorCommand { mode, target } => {
            if out.len() < 5 { return Err(DecodeError::Truncated); }
            out[0] = *mode;
            put_u32(&mut out[1..5], target.to_bits());
            Ok(5)
        }
    }
}

fn decode_data(cmd: Command, is_controller: bool, b: &[u8]) -> Result<ProtocolData, DecodeError> {
    use Command::*;
    use ProtocolData::*;
    let need = |n: usize| if b.len() < n { Err(DecodeError::Truncated) } else { Ok(()) };
    Ok(match cmd {
        Noop | SetServo | Ack | FirmwareUpdate | UserStoreUpdate
        | EraseUserStore | NetworkReset | SaveMotorParams => Empty,
        StreamWrite => {
            // Final fragment may be short — pad with 0xFF and rely on the
            // receiver tracking total bytes via `StreamStart::stream_length`.
            let n = b.len().min(7);
            let mut d = [0xFFu8; 7];
            d[..n].copy_from_slice(&b[..n]);
            StreamFragment(d)
        }
        GetInfo => { need(7)?; Info {
            serial: get_u32(&b[0..4]), fw_major: b[4], fw_minor: b[5], fw_patch: b[6] } }
        GetInfoExt => { need(2)?; InfoExt { flags: b[0], temperature: b[1] } }
        GetPosition | SetPosition => { need(2)?; Position { value: get_u16(&b[0..2]) } }
        GetStatus => { need(1)?; Status(StatusBits::from_byte(b[0])) }
        GetAnalog => { need(4)?; Analog { a0: get_u16(&b[0..2]), a1: get_u16(&b[2..4]) } }
        GetServo  => { need(5)?; Servo { s0: get_u16(&b[0..2]), s1: get_u16(&b[2..4]),
            update_flag: b[4] } }
        GetLed | SetLed => { need(3)?; Led { sys: b[0], stat: b[1], update_flag: b[2] } }
        GetMotor | SetMotor => Motor(MotorBits::from_bytes(b)?),
        GetFoc | SetFoc => { need(4)?; Foc { foc_1: b[0], foc_2: b[1], foc_3: b[2], en: b[3] } }
        StreamStart => { need(4)?; ProtocolData::StreamStart {
            stream_length: get_u32(&b[0..4]) } }
        StreamRead => {
            need(7)?;
            let mut d = [0u8; 7];
            d.copy_from_slice(&b[0..7]);
            StreamFragment(d)
        }
        StreamCommit => { need(4)?; ProtocolData::StreamCommit {
            crc32: get_u32(&b[0..4]) } }
        RevokeConfig => { need(4)?; ProtocolData::RevokeConfig { serial_no: get_u32(&b[0..4]) } }
        Error => { need(5)?; ProtocolData::Error { code: b[0], message: get_u32(&b[1..5]) } }
        GetMotorParam | SetMotorParam => {
            need(5)?;
            MotorParam { index: b[0], value: f32::from_bits(get_u32(&b[1..5])) }
        }
        RunCalibration => {
            if is_controller {
                need(5)?;
                CalibrationReq {
                    freq_dhz:   get_u16(&b[0..2]),
                    dur_ms:     get_u16(&b[2..4]),
                    valign_pct: b[4],
                }
            } else {
                need(7)?;
                CalibrationResult {
                    pole_pairs:  f32::from_bits(get_u32(&b[0..4])),
                    zero_offset: get_u16(&b[4..6]),
                    flags:       b[6],
                }
            }
        }
        SetMotorCommand => {
            need(5)?;
            MotorCommand { mode: b[0], target: f32::from_bits(get_u32(&b[1..5])) }
        }
        Invalid => return Err(DecodeError::InvalidCommand),
    })
}

// ---- Public encode / decode API ------------------------------------------

impl Message {
    /// Build a CAN frame for this message.
    pub fn encode(&self) -> Result<CanFrame, DecodeError> {
        let mut buf = [0u8; CAN_MAX_DLEN];
        let (id, len) = match self {
            Message::DiscoveryReq { serial, previous_id } => {
                if buf.len() < 5 { return Err(DecodeError::Truncated); }
                put_u32(&mut buf[0..4], *serial);
                buf[4] = *previous_id;
                (DISCOVERY_REQUEST_ID, 5)
            }
            Message::DiscoveryAssign { serial, assigned_id } => {
                put_u32(&mut buf[0..4], *serial);
                buf[4] = *assigned_id;
                (DISCOVERY_ASSIGN_ID, 5)
            }
            Message::Control { device_id, is_controller, cmd, data } => {
                buf[0] = cmd.to_u8() | (if *is_controller { CONTROLLER_BIT } else { 0 });
                let n = encode_data(data, &mut buf[1..])?;
                (DEVICE_BASE_ID + (*device_id as u16), 1 + n)
            }
            Message::Controller(b, len) => {
                let n = (*len as usize).min(CAN_MAX_DLEN);
                buf[..n].copy_from_slice(&b[..n]);
                (CONTROLLER_ID, n)
            }
        };
        Ok(CanFrame {
            id,
            len: len as u8,
            rtr: 0,
            data: buf,
        })
    }

    /// Parse a CAN frame. Returns `Ok(None)` for IDs we don't decode
    /// (currently the controller channel and reserved IDs 3..=7).
    pub fn decode(frame: &CanFrame) -> Result<Option<Self>, DecodeError> {
        let len = frame.len as usize;
        if len == 0 { return Err(DecodeError::Empty); }
        let bytes = &frame.data[..len.min(CAN_MAX_DLEN)];

        match frame.id {
            CONTROLLER_ID => {
                let mut buf = [0u8; CAN_MAX_DLEN];
                buf[..bytes.len()].copy_from_slice(bytes);
                Ok(Some(Message::Controller(buf, bytes.len() as u8)))
            }
            DISCOVERY_REQUEST_ID => {
                if bytes.len() < 5 { return Err(DecodeError::Truncated); }
                Ok(Some(Message::DiscoveryReq {
                    serial:      get_u32(&bytes[0..4]),
                    previous_id: bytes[4],
                }))
            }
            DISCOVERY_ASSIGN_ID => {
                if bytes.len() < 5 { return Err(DecodeError::Truncated); }
                Ok(Some(Message::DiscoveryAssign {
                    serial:      get_u32(&bytes[0..4]),
                    assigned_id: bytes[4],
                }))
            }
            id if id < DEVICE_BASE_ID => Ok(None),  // reserved
            id => {
                let cmd_byte      = bytes[0];
                let is_controller = cmd_byte & CONTROLLER_BIT != 0;
                let cmd           = Command::from_u8(cmd_byte);
                if cmd == Command::Invalid { return Err(DecodeError::InvalidCommand); }
                let data = decode_data(cmd, is_controller, &bytes[1..])?;
                Ok(Some(Message::Control {
                    device_id: (id - DEVICE_BASE_ID) as u8,
                    is_controller,
                    cmd,
                    data,
                }))
            }
        }
    }
}

// ---- Tests (host-only) ---------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn discovery_round_trip() {
        let m = Message::DiscoveryReq { serial: 0xDEAD_BEEF, previous_id: 0x42 };
        let f = m.encode().unwrap();
        assert_eq!(f.id, DISCOVERY_REQUEST_ID);
        assert_eq!(Message::decode(&f).unwrap(), Some(m));
    }

    #[test]
    fn control_round_trip() {
        let m = Message::Control {
            device_id: 5,
            is_controller: true,
            cmd: Command::SetPosition,
            data: ProtocolData::Position { value: 0x1234 },
        };
        let f = m.encode().unwrap();
        assert_eq!(f.id, DEVICE_BASE_ID + 5);
        assert_eq!(f.data[0] & CONTROLLER_BIT, CONTROLLER_BIT);
        assert_eq!(Message::decode(&f).unwrap(), Some(m));
    }

    #[test]
    fn status_bits() {
        let s = StatusBits { endstop0: true, fault: true, mag: 0xA, ..Default::default() };
        assert_eq!(StatusBits::from_byte(s.to_byte()), s);
    }

    #[test]
    fn invalid_command_rejected() {
        let f = CanFrame { id: DEVICE_BASE_ID, len: 1, rtr: 0, data: [0x7F, 0,0,0,0,0,0,0] };
        assert_eq!(Message::decode(&f), Err(DecodeError::InvalidCommand));
    }

    #[test]
    fn stream_start_fits_in_can() {
        let m = Message::Control {
            device_id: 5, is_controller: true,
            cmd: Command::StreamStart,
            data: ProtocolData::StreamStart { stream_length: 0x1234_5678 },
        };
        let f = m.encode().unwrap();
        assert_eq!(f.len, 5);  // 1 cmd + 4 length bytes
        assert_eq!(Message::decode(&f).unwrap(), Some(m));
    }

    #[test]
    fn stream_write_round_trip() {
        let m = Message::Control {
            device_id: 5, is_controller: true,
            cmd: Command::StreamWrite,
            data: ProtocolData::StreamFragment([0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77]),
        };
        let f = m.encode().unwrap();
        assert_eq!(f.len, 8);
        assert_eq!(Message::decode(&f).unwrap(), Some(m));
    }

    #[test]
    fn stream_write_short_fragment_pads() {
        // Trailing fragment with only 3 real bytes \u2014 decoder pads with 0xFF.
        let f = CanFrame {
            id: DEVICE_BASE_ID + 5, len: 4, rtr: 0,
            data: [Command::StreamWrite as u8 | CONTROLLER_BIT, 0xAA, 0xBB, 0xCC, 0,0,0,0],
        };
        if let Some(Message::Control { data: ProtocolData::StreamFragment(d), .. })
            = Message::decode(&f).unwrap()
        {
            assert_eq!(d, [0xAA, 0xBB, 0xCC, 0xFF, 0xFF, 0xFF, 0xFF]);
        } else {
            panic!("expected StreamWrite/StreamFragment");
        }
    }

    #[test]
    fn stream_commit_round_trip() {
        let m = Message::Control {
            device_id: 5, is_controller: true,
            cmd: Command::StreamCommit,
            data: ProtocolData::StreamCommit { crc32: 0xDEAD_BEEF },
        };
        let f = m.encode().unwrap();
        assert_eq!(f.len, 5); // 1 cmd + 4 crc
        assert_eq!(Message::decode(&f).unwrap(), Some(m));
    }

    #[test]
    fn motor_command_round_trip() {
        let m = Message::Control {
            device_id: 5, is_controller: true,
            cmd: Command::SetMotorCommand,
            data: ProtocolData::MotorCommand { mode: 1, target: -2.5 },
        };
        let f = m.encode().unwrap();
        assert_eq!(f.len, 6); // 1 cmd + 1 mode + 4 f32
        assert_eq!(Message::decode(&f).unwrap(), Some(m));
    }
}
