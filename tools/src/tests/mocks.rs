struct MockCommManager {
    can_tx: Sender<CanDriveMessage>,
    can_rx: Receiver<CanDriveMessage>,
    local_rx: Receiver<LocalCommands>,
    local_tx: Sender<LocalCommands>,
    text_rx: Receiver<CanDriveMessage>, // Sink can_tx
    text_tx: Sender<CanDriveMessage>,   // Source can_rx
}

struct MockDevice {
    model: Device,
    previous_id: u8,
    text_tx: Sender<CanDriveMessage>,   // Connect to can_rx
    text_rx: Receiver<CanDriveMessage>, // Connect to can_tx
}

impl MockDevice {
    fn new(
        serial: u32,
        previous_id: u8,
        text_rx: Receiver<CanDriveMessage>,
        text_tx: Sender<CanDriveMessage>,
    ) -> MockDevice {
        let mut model = Device::new();
        MockDevice { model, previous_id }
    }

    // Enque a message to be read by can_rx
    fn send_reply(&self, cmd: Commands, data: ProtocolData) {
        text_tx
            .send(CanDriveMessage::Control {
                id: DeviceId::new(self.previous_id),
                is_controller: true,
                cmd: cmd,
                data: data,
            })
            .unwrap();
    }

    // Simulate device handling a message and responding
    fn process_message(&mut self, msg: CanDriveMessage) {
        // Always succeed and update the model
        match msg {
            CanDriveMessage::Control {
                id,
                is_controller,
                cmd,
                data,
            } => {
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
                            self.send_reply(
                                cmd,
                                ProtocolData::GetInfo {
                                    serial: self.model.serial,
                                    fw_major: self.model.fw_major,
                                    fw_minor: self.model.fw_minor,
                                    fw_patch: self.model.fw_patch,
                                },
                            );
                        }
                    }
                    Commands::GetInfoExt => {
                        self.send_reply(
                            cmd,
                            ProtocolData::InfoExt {
                                flags: 0,
                                temperature: 0,
                            },
                        );
                    }
                    Commands::GetPosition => {
                        self.send_reply(
                            cmd,
                            ProtocolData::Position {
                                value: self.model.position.reported_value,
                            },
                        );
                    }
                    Commands::SetPosition => {
                        if let ProtocolData::Position { value } = data {
                            self.model.position.target_value = *value;
                        }
                        self.send_reply(cmd, ProtocolData::Ack);
                    }
                    Commands::GetStatus => {
                        self.send_reply(cmd, ProtocolData::Status(self.model.status));
                    }
                    Commands::GetAnalog => {
                        self.send_reply(
                            cmd,
                            ProtocolData::Analog {
                                a0: self.model.analog0.reported_value,
                                a1: self.model.analog1.reported_value,
                            },
                        );
                    }
                    Commands::GetServo => {
                        self.send_reply(
                            cmd,
                            ProtocolData::Servo {
                                s0: self.model.servo0.reported_value,
                                s1: self.model.servo1.reported_value,
                                update_flag: 0x03,
                            },
                        );
                    }
                    Commands::SetServo => {
                        if let ProtocolData::Servo {
                            s0,
                            s1,
                            update_flag,
                        } = data
                        {
                            if *update_flag & 0x01 == 0x01 {
                                self.model.servo0.target_value = *s0;
                            }
                            if *update_flag & 0x02 == 0x02 {
                                self.model.servo1.target_value = *s1;
                            }
                        }
                    }
                    Commands::GetLed => {
                        self.send_reply(
                            cmd,
                            ProtocolData::Led {
                                sys: self.model.led_sys.reported_value,
                                stat: self.model.led_stat.reported_value,
                                update_flag: 0x03,
                            },
                        );
                    }
                    Commands::SetLed => {
                        if let ProtocolData::Led {
                            sys,
                            stat,
                            update_flag,
                        } = data
                        {
                            if *update_flag & 0x01 == 0x01 {
                                self.model.led_stat.target_value = *stat;
                            }
                            if *update_flag & 0x02 == 0x02 {
                                self.model.led_sys.target_value = *sys;
                            }
                        }
                    }
                    Commands::GetMotor => {
                        if let ProtocolData::Motor(motor) = data {
                            self.model.motor = *motor;
                        }
                    }
                    Commands::SetMotor => {
                        if let ProtocolData::Motor(motor) = data {
                            self.model.motor = *motor;
                        }
                    }
                    Commands::GetFOC => self.send_reply(
                        cmd,
                        ProtocolData::FOC {
                            foc_1: self.model.foc1.reported_value,
                            foc_2: self.model.foc2.reported_value,
                            foc_3: self.model.foc3.reported_value,
                            en: self.model.foc_en.reported_value,
                        },
                    ),
                    Commands::SetFOC => {
                        if let ProtocolData::FOC {
                            foc_1,
                            foc_2,
                            foc_3,
                            en,
                        } = data
                        {
                            self.model.foc1.target_value = *foc_1;
                            self.model.foc2.target_value = *foc_2;
                            self.model.foc3.target_value = *foc_3;
                            self.model.foc_en.target_value = *en;
                        }
                    }
                    Commands::StreamStart => {
                        // TODO: check read/write flags
                        if let ProtocolData::StreamStart {
                            stream_target,
                            stream_length,
                            flags,
                        } = data
                        {
                            self.send_reply(cmd, ProtocolData::Ack);
                        }
                    }
                    Commands::StreamData => {
                        // TODO: check read/write flags
                        if let ProtocolData::StreamFragment { sequence, data } = data {
                            self.send_reply(cmd, ProtocolData::Ack);
                        }
                    }
                    Commands::Ack => {
                        // Nothing to do
                    }
                    Commands::StartFwUpdate => {
                        // TODO: check read/write flags
                        self.send_reply(cmd, ProtocolData::Ack);
                    }
                    Commands::FWUpdate => {
                        // TODO: check read/write flags
                        self.send_reply(cmd, ProtocolData::Ack);
                    }
                    Commands::NetworkReset => {
                        // TODO: check read/write flags
                        self.model.id = INVALID_DEVICE;
                    }
                    Commands::OverwriteUserStore => {
                        // TODO: check read/write flags
                        self.send_reply(cmd, ProtocolData::Ack);
                    }
                    Commands::EraseUserStore => {
                        // TODO: check read/write flags
                        self.send_reply(cmd, ProtocolData::Ack);
                    }
                    Commands::RevokeConfig => {
                        self.model.id = INVALID_DEVICE;
                    }
                    Commands::Error => {
                        // Nothign to do
                    }
                    Commands::Invalid => {
                        self.send_reply(
                            cmd,
                            ProtocolData::Error {
                                code: 0xFF,
                                message: 0,
                            },
                        );
                    }
                };
            }
        }
    }
}

impl MockCommManager {
    fn send(&self, msg: CanDriveMessage) {
        self.can_tx.send(msg).unwrap();
    }

    fn receive(&self) -> Result<CanDriveMessage, String> {
        self.can_rx.recv().unwrap();
    }

    fn init(&mut self) {
        utils::init_message_queues(self.can_tx.clone(), self.can_rx.clone(), local_rx.clone());
    }
}
