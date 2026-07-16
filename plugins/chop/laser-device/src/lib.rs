//! Laser Device CHOP: streams laser point data from CHOP channels to laser
//! DAC hardware (Ether Dream, LaserCube, IDN — via the `laser-dac` crate) or
//! to PONK network receivers such as MadMapper (via `ponk-protocol`).
//!
//! Input convention matches TouchDesigner's native laser workflow: one sample
//! per point, channels named x/y (and optionally r/g/b/i), resolved
//! case-insensitively with a positional fallback (see `mapping`).
//!
//! Laser safety: the Active toggle defaults to off, x/y are hard-clamped to
//! [-1, 1] and a blank frame is emitted on teardown or input loss. Galvo
//! velocity/density limiting is the DAC's (or laser-dac's) responsibility —
//! this plugin does not implement scanner-safety profiles.

mod device;
mod mapping;

use device::{DacBackend, DeviceEntry, LaserBackend, PonkBackend};
use mapping::{MapOptions, Point2};
use td_rs_chop::sop::Color;
use td_rs_chop::*;
use td_rs_derive::{Param, Params};

#[derive(Param, Default, Clone, Debug, PartialEq)]
enum OutputBackend {
    #[default]
    Hardware,
    PonkNetwork,
}

#[derive(Params)]
struct LaserDeviceParams {
    #[param(label = "Active", page = "Device")]
    active: bool,
    #[param(label = "Backend", page = "Device")]
    backend: OutputBackend,
    #[param(label = "Device", page = "Device")]
    device: DynamicMenuParam,
    #[param(label = "Refresh Devices", page = "Device")]
    refresh: Pulse,

    #[param(
        label = "Points Per Second",
        page = "Output",
        min = 1000.0,
        max = 200000.0
    )]
    pps: u32,
    #[param(label = "Intensity", page = "Output", min = 0.0, max = 1.0)]
    intensity: f32,
    #[param(label = "Scale", page = "Output", min = 0.0, max = 2.0)]
    scale: f32,
    #[param(label = "Default Color", page = "Output")]
    default_color: Color,

    #[param(label = "Address", page = "Network")]
    address: String,
    #[param(label = "Sender Name", page = "Network")]
    sender_name: String,
}

enum Connection {
    Idle,
    Connected {
        backend: Box<dyn LaserBackend>,
        /// Snapshot of the connection-relevant params, for change detection.
        config_key: String,
        frames_sent: u64,
        last_points: usize,
    },
    /// A failed connection is retried only when the config changes or the
    /// user pulses Refresh — never every cook, which would hitch every frame.
    Failed {
        config_key: String,
    },
}

/// What went wrong during a cook's send step.
enum CookIssue {
    /// Input data problem — warn but stay connected.
    Input(String),
    /// Device problem — drop to Failed so Refresh/param changes retry.
    Device(String),
}

pub struct LaserDeviceChop {
    params: LaserDeviceParams,
    conn: Connection,
    devices: Vec<DeviceEntry>,
    scanned_once: bool,
    force_reconnect: bool,
    points_buf: Vec<Point2>,
}

impl LaserDeviceChop {
    fn config_key(&self) -> String {
        format!(
            "{:?}|{}|{}|{}|{}",
            self.params.backend,
            self.params.device.0.as_deref().unwrap_or(""),
            self.params.pps,
            self.params.address,
            self.params.sender_name,
        )
    }

    fn refresh_devices(&mut self) {
        self.scanned_once = true;
        match device::discover() {
            Ok(devices) => self.devices = devices,
            Err(e) => self.set_warning(&e),
        }
    }

    fn connect_backend(&self) -> Result<Box<dyn LaserBackend>, String> {
        match self.params.backend {
            OutputBackend::Hardware => {
                let token = self
                    .params
                    .device
                    .0
                    .as_deref()
                    .filter(|t| !t.is_empty())
                    .ok_or("no device selected — press Refresh Devices and pick one")?;
                let entry = self
                    .devices
                    .iter()
                    .find(|e| e.token == token)
                    .ok_or_else(|| format!("device '{token}' not found — press Refresh Devices"))?;
                Ok(Box::new(DacBackend::connect(&entry.id, self.params.pps)?))
            }
            OutputBackend::PonkNetwork => Ok(Box::new(PonkBackend::connect(
                &self.params.address,
                &self.params.sender_name,
            )?)),
        }
    }

    /// Drive the connection state machine from the current params. All
    /// connects/teardowns happen here, once per cook at most.
    fn reconcile(&mut self) {
        let key = self.config_key();
        let want = self.params.active;

        let teardown = matches!(
            &self.conn,
            Connection::Connected { config_key, .. } if !want || *config_key != key
        );
        if teardown {
            if let Connection::Connected { mut backend, .. } =
                std::mem::replace(&mut self.conn, Connection::Idle)
            {
                let _ = backend.blank();
                // Dropping the backend stops its session.
            }
            self.set_info("");
        }

        if !want {
            self.conn = Connection::Idle;
            return;
        }

        let should_connect = match &self.conn {
            Connection::Idle => true,
            Connection::Failed { config_key } => self.force_reconnect || *config_key != key,
            Connection::Connected { .. } => false,
        };
        if !should_connect {
            return;
        }
        self.force_reconnect = false;

        match self.connect_backend() {
            Ok(backend) => {
                self.set_info(&backend.describe());
                self.set_warning("");
                self.conn = Connection::Connected {
                    backend,
                    config_key: key,
                    frames_sent: 0,
                    last_points: 0,
                };
            }
            Err(e) => {
                self.set_warning(&e);
                self.conn = Connection::Failed { config_key: key };
            }
        }
    }
}

impl OpNew for LaserDeviceChop {
    fn new(_info: NodeInfo) -> Self {
        Self {
            params: LaserDeviceParams {
                // Never emit on node creation: the user must arm explicitly.
                active: false,
                backend: OutputBackend::Hardware,
                device: DynamicMenuParam::default(),
                refresh: Pulse,
                pps: 30_000,
                intensity: 1.0,
                scale: 1.0,
                default_color: (1.0, 1.0, 1.0, 1.0).into(),
                address: format!(
                    "{}.{}.{}.{}:{}",
                    ponk_protocol::MULTICAST_ADDR[0],
                    ponk_protocol::MULTICAST_ADDR[1],
                    ponk_protocol::MULTICAST_ADDR[2],
                    ponk_protocol::MULTICAST_ADDR[3],
                    ponk_protocol::DEFAULT_PORT
                ),
                sender_name: "TouchDesigner".to_string(),
            },
            conn: Connection::Idle,
            devices: Vec::new(),
            scanned_once: false,
            force_reconnect: false,
            points_buf: Vec::new(),
        }
    }
}

impl OpInfo for LaserDeviceChop {
    const OPERATOR_TYPE: &'static str = "Laserdevice";
    const OPERATOR_LABEL: &'static str = "Laser Device";
    const MIN_INPUTS: usize = 1;
    const MAX_INPUTS: usize = 1;
    // Populate the device menu's discovery cache before the user first opens
    // it (build_dynamic_menu takes &self, so it cannot scan itself).
    const COOK_ON_START: bool = true;
}

impl Op for LaserDeviceChop {
    fn params_mut(&mut self) -> Option<Box<&mut dyn OperatorParams>> {
        Some(Box::new(&mut self.params))
    }

    fn pulse_pressed(&mut self, name: &str) {
        if name == "Refresh" {
            self.refresh_devices();
            self.force_reconnect = true;
        }
    }
}

impl Chop for LaserDeviceChop {
    fn execute(&mut self, output: &mut ChopOutput, inputs: &OperatorInputs<ChopInput>) {
        let hardware = self.params.backend == OutputBackend::Hardware;
        let params = inputs.params();
        params.enable_param("Device", hardware);
        params.enable_param("Refresh", hardware);
        params.enable_param("Pps", hardware);
        params.enable_param("Address", !hardware);
        params.enable_param("Sendername", !hardware);

        if !self.scanned_once {
            self.refresh_devices();
        }
        self.reconcile();

        // Stream one frame. Warnings are applied after the &mut borrow of
        // self.conn ends (set_warning needs &mut self).
        let mut issue: Option<CookIssue> = None;
        let mut sent_points: Option<usize> = None;
        if let Connection::Connected { backend, .. } = &mut self.conn {
            match inputs.input(0) {
                Some(input) if input.num_channels() >= 2 => {
                    let names: Vec<&str> = (0..input.num_channels())
                        .map(|i| input.channel_name(i))
                        .collect();
                    match mapping::resolve_channels(&names) {
                        Ok(map) => {
                            let channels: Vec<&[f32]> = (0..input.num_channels())
                                .map(|i| input.channel(i))
                                .collect();
                            let opts = MapOptions {
                                scale: self.params.scale,
                                intensity: self.params.intensity,
                                default_rgb: [
                                    self.params.default_color.r,
                                    self.params.default_color.g,
                                    self.params.default_color.b,
                                ],
                            };
                            mapping::build_points(
                                &channels,
                                input.num_samples(),
                                &map,
                                &opts,
                                &mut self.points_buf,
                            );
                            match backend.send(&self.points_buf) {
                                Ok(()) => sent_points = Some(self.points_buf.len()),
                                Err(e) => issue = Some(CookIssue::Device(e)),
                            }
                        }
                        Err(e) => {
                            let _ = backend.blank();
                            issue = Some(CookIssue::Input(e));
                        }
                    }
                }
                _ => {
                    let _ = backend.blank();
                    issue = Some(CookIssue::Input(
                        "input CHOP with at least x and y channels is required".to_string(),
                    ));
                }
            }
        }

        match issue {
            None => {
                if let Some(n) = sent_points {
                    if let Connection::Connected {
                        frames_sent,
                        last_points,
                        ..
                    } = &mut self.conn
                    {
                        *frames_sent += 1;
                        *last_points = n;
                    }
                    self.set_warning("");
                }
            }
            Some(CookIssue::Input(e)) => self.set_warning(&e),
            Some(CookIssue::Device(e)) => {
                self.set_warning(&e);
                let key = self.config_key();
                self.conn = Connection::Failed { config_key: key };
                self.set_info("");
            }
        }

        // Status channels.
        let (connected, points, frames) = match &self.conn {
            Connection::Connected {
                backend,
                last_points,
                frames_sent,
                ..
            } => (
                backend.is_connected(),
                *last_points as f32,
                *frames_sent as f32,
            ),
            _ => (false, 0.0, 0.0),
        };
        if output.num_channels() >= 4 && output.num_samples() >= 1 {
            output[0][0] = connected as u8 as f32;
            output[1][0] = self.params.active as u8 as f32;
            output[2][0] = points;
            output[3][0] = frames;
        }
    }

    fn general_info(&self, _inputs: &OperatorInputs<ChopInput>) -> ChopGeneralInfo {
        ChopGeneralInfo {
            cook_every_frame: true,
            cook_every_frame_if_asked: true,
            timeslice: false,
            input_match_index: 0,
        }
    }

    fn output_info(&self, _inputs: &OperatorInputs<ChopInput>) -> Option<ChopOutputInfo> {
        Some(ChopOutputInfo {
            num_channels: 4,
            num_samples: 1,
            start_index: 0,
            ..Default::default()
        })
    }

    fn channel_name(&self, index: usize, _inputs: &OperatorInputs<ChopInput>) -> String {
        ["connected", "active", "points", "frames"]
            .get(index)
            .unwrap_or(&"")
            .to_string()
    }

    fn build_dynamic_menu(
        &self,
        _inputs: &OperatorInputs<ChopInput>,
        menu_info: &mut DynamicMenuInfo,
    ) {
        if menu_info.param_name() == "Device" {
            for entry in &self.devices {
                menu_info.add_menu_entry(&entry.token, &entry.label);
            }
        }
    }
}

chop_plugin!(LaserDeviceChop);
