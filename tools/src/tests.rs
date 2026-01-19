
struct MockCommManager {
    can_tx: Sender<CanDriveMessage>,
    can_rx: Receiver<CanDriveMessage>,
}

struct MockDevice
{
    model: Device,
}

impl MockDevice
{
    fn new() -> MockDevice
    {
        MockDevice {
            model: Device::new(),
        }
    }

    fn process_message(&mut self, msg: CanDriveMessage)
    {
        // Always succeed and update the model
        match msg {
            CanDriveMessage::Control { id, is_controller, cmd, data } => {
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
                                self.model.fw_major = *fw_major;
                                self.model.fw_minor = *fw_minor;
                                self.model.fw_patch = *fw_patch;
                                self.send_reply(cmd, ProtocolData::GetInfo{serial: self.model.serial, fw_major: self.model.fw_major, fw_minor: self.model.fw_minor, fw_patch: self.model.fw_patch});
                            }
                        }
                        Commands::GetPosition => {
                            if let ProtocolData::Position { value } = data {
                                self.model.position.reported_value = *value;
                            }
                        }
                        Commands::GetStatus => {
                            if let ProtocolData::Status(status) = data {
                                self.model.status = *status;
                            }
                        }
                        Commands::GetAnalog => {
                            if let ProtocolData::Analog { a0, a1 } = data {
                                self.model.analog0.reported_value = *a0;
                                self.model.analog1.reported_value = *a1;
                            }
                        }
                        Commands::GetServo => {
                            if let ProtocolData::Servo { s0, s1, update_flag } = data {
                                if *update_flag & 0x01 == 0x01 {
                                    self.model.servo0.target_value = *s0;
                                }
                                if *update_flag & 0x02 == 0x02 {
                                    self.model.servo1.target_value = *s1;
                                }
                            }
                        }
                        Commands::GetLed => {
                            if let ProtocolData::Led { sys, stat, update_flag } = data {
                                if *update_flag & 0x01 == 0x01 {
                                    self.model.led_stat.reported_value = *stat;
                                }
                                if *update_flag & 0x02 == 0x02 {
                                    self.model.led_sys.reported_value = *sys;
                                }
                            }
                        }
                        Commands::GetMotor => {
                            if let ProtocolData::Motor(motor) = data {
                                self.model.motor = *motor;
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
                                self.model.foc1.reported_value = *foc_1;
                                self.model.foc2.reported_value = *foc_2;
                                self.model.foc3.reported_value = *foc_3;
                                self.model.foc_en.reported_value = *en;
                            }
                        }
                        _ => {
                            // unimplemented
                            return Err("Warning: Unhandled device command".to_string());
                        }
                    }
                    // Update complete
                    Ok(())
                },
            _ => {
                // unimplemented
                return Err("Warning: Unhandled device command".to_string());
            }
        }
    }

    fn send_reply(&self, cmd: Commands, data: ProtocolData, reply_to: Sender<CanDriveMessage>)
    {
        let can_rx = get_can_rx_queue();
    }
}

mod tests
{
    use super::*;
    use std::thread;
    use std::time::Duration;

    #[test]
    fn test_discovery_flow()
    {
        let (can_tx, can_rx) = crossbeam_channel::unbounded();
        let (local_tx, local_rx) = crossbeam_channel::unbounded();
        let device_model = MockDevice::new();
    }

    #[test]
    fn test_controller_flow()
    {

    }

    #[test]
    fn test_local_commands()
    {
        let commands_to_test = [
            (LocalCommands::ListDevices, None), // Just expect Ok() this outputs to stdout
            LocalCommands::ResetNetwork, // Should reset the network
            LocalCommands::SetPosition(DeviceId::new(0), 0),
            LocalCommands::GetPosition(DeviceId::new(0)),
            LocalCommands::GetInfo(DeviceId::new(0)),
            LocalCommands::GetStatus(DeviceId::new(0)),
            LocalCommands::SetSysLed(DeviceId::new(0), 0),
            LocalCommands::SetStatLed(DeviceId::new(0), 0),
            LocalCommands::GetLeds(DeviceId::new(0)),
            LocalCommands::GetMotor(DeviceId::new(0)),
            LocalCommands::GetAnalog(DeviceId::new(0)),
            LocalCommands::GetServo(DeviceId::new(0)),
            LocalCommands::SetServo0(DeviceId::new(0), 0),
        ];
        let (can_tx, can_rx) = std::sync::mpsc::channel();
        let mut local_command_parser = LocalCommandParser::new();
        let local_rx = local_command_parser.get_command_queue();
        init_message_queues(can_tx.clone(), can_rx.clone(), local_rx.clone());
        let mut device_manager = DeviceManager::new(&can_tx, &can_rx, &local_rx);
        device_manager.start();
        thread::sleep(Duration::from_millis(100));
        device_manager.stop();
    }

    #[test]
    fn test_device_manager()
    {
        let (can_tx, can_rx) = std::sync::mpsc::channel();
        let mut local_command_parser = LocalCommandParser::new();
        let local_rx = local_command_parser.get_command_queue();
        init_message_queues(can_tx.clone(), can_rx.clone(), local_rx.clone());
        let mut device_manager = DeviceManager::new(&can_tx, &can_rx, &local_rx);
        device_manager.start();
        thread::sleep(Duration::from_millis(100));
        device_manager.stop();
    }
}