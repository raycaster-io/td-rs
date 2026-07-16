//! Laser Device CHOP: streams laser point data from CHOP channels to laser
//! DAC hardware (Ether Dream, LaserCube, IDN — via the `laser-dac` crate) or
//! to PONK network receivers such as MadMapper (via `ponk-protocol`).
//!
//! Input convention matches TouchDesigner's native laser workflow: one sample
//! per point, channels named x/y (and optionally r/g/b/i), resolved
//! case-insensitively with a positional fallback (see `mapping`).
//!
//! Laser safety: the Active toggle defaults to off, x/y are hard-clamped to
//! [-1, 1] (non-finite coordinates are blanked), a blank frame is emitted on
//! input loss, and hardware teardown disarms the session and lets the device
//! queue drain into blanks before stopping. Galvo velocity/density limiting
//! is the DAC's (or laser-dac's) responsibility — this plugin does not
//! implement scanner-safety profiles.
//!
//! Threading: DAC discovery, connecting, and teardown all block (network
//! scans, buffer flushes, thread joins), so they run on background threads
//! and hand results back over channels; `execute()` only ever polls.

mod device;
mod mapping;

use std::sync::mpsc;

use device::{DacBackend, DeviceEntry, LaserBackend, PonkBackend, SendError};
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

/// Snapshot of the parameters the ACTIVE backend actually connects with.
/// Scoping fields per backend means editing PONK params never tears down a
/// live hardware session (and vice versa), and comparing structs instead of
/// a joined string cannot alias across field boundaries.
#[derive(Clone, PartialEq, Debug)]
enum ConnConfig {
    Hardware {
        device: String,
        pps: u32,
    },
    Ponk {
        address: String,
        sender_name: String,
    },
}

type ConnectResult = Result<Box<dyn LaserBackend>, String>;

enum Connection {
    Idle,
    /// A background thread is opening the backend.
    Connecting {
        rx: mpsc::Receiver<ConnectResult>,
        config: ConnConfig,
    },
    /// An abandoned connect (config changed mid-flight) still owns the
    /// device until its thread resolves — single-client DACs refuse a second
    /// connection, so no new connect starts until this drains.
    Draining {
        rx: mpsc::Receiver<ConnectResult>,
    },
    Connected {
        backend: Box<dyn LaserBackend>,
        config: ConnConfig,
        frames_sent: u64,
        last_points: usize,
    },
    /// A failed connection is retried when the config changes, the user
    /// pulses Refresh, or a device scan completes — never every cook, which
    /// would spawn a connect attempt per frame.
    Failed {
        config: ConnConfig,
    },
}

pub struct LaserDeviceChop {
    params: LaserDeviceParams,
    conn: Connection,
    devices: Vec<DeviceEntry>,
    /// In-flight background device scan, if any.
    scan_rx: Option<mpsc::Receiver<Result<Vec<DeviceEntry>, String>>>,
    scanned_once: bool,
    /// Retry ticket for a Failed connection. Set by the Refresh pulse and by
    /// completed device scans; cleared when consumed by a connect attempt or
    /// once the connection is healthy. Deliberately NOT cleared while a
    /// connect is in flight, so a pulse/scan that lands mid-Connecting still
    /// retries if that connect fails.
    force_reconnect: bool,
    points_buf: Vec<Point2>,
    // Per-instance status strings: the framework's default set_info/
    // set_warning/set_error route through process-wide statics shared by
    // every node of this plugin type, so two Laser Device nodes would
    // overwrite each other's messages.
    info_msg: String,
    warning_msg: String,
    error_msg: String,
}

impl LaserDeviceChop {
    fn conn_config(&self) -> ConnConfig {
        match self.params.backend {
            OutputBackend::Hardware => ConnConfig::Hardware {
                device: self.params.device.0.clone().unwrap_or_default(),
                pps: self.params.pps,
            },
            OutputBackend::PonkNetwork => ConnConfig::Ponk {
                address: self.params.address.clone(),
                sender_name: self.params.sender_name.clone(),
            },
        }
    }

    /// Retire a backend off the cook thread: shutdown() disarms, waits for
    /// the device queue to drain into blanks, then stops and joins the
    /// scheduler — seconds of blocking in the worst (mid-reconnect) case.
    fn teardown(backend: Box<dyn LaserBackend>) {
        std::thread::spawn(move || backend.shutdown());
    }

    /// Kick off a background device scan unless one is already running.
    /// Discovery blocks for ~2s of sequential network scans, so it must
    /// never run on the cook thread.
    fn refresh_devices(&mut self) {
        if self.scan_rx.is_some() {
            return;
        }
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let _ = tx.send(device::discover());
        });
        self.scan_rx = Some(rx);
    }

    /// Collect a finished background scan, if any.
    fn poll_devices(&mut self) {
        let Some(rx) = self.scan_rx.take() else {
            return;
        };
        match rx.try_recv() {
            Ok(Ok(devices)) => {
                self.devices = devices;
                // Fresh scan data may make an unresolvable device selection
                // connectable (a saved .toe found by the startup scan, a DAC
                // that changed IP). Arm a retry unless already healthy; if a
                // connect is in flight the ticket survives until it either
                // succeeds (cleared below) or fails (consumed by the retry).
                if self.params.active && !matches!(self.conn, Connection::Connected { .. }) {
                    self.force_reconnect = true;
                }
            }
            Ok(Err(e)) => self.set_warning(&e),
            Err(mpsc::TryRecvError::Empty) => self.scan_rx = Some(rx),
            Err(mpsc::TryRecvError::Disconnected) => {
                self.set_warning("device scan thread terminated unexpectedly");
            }
        }
    }

    /// Start opening a backend on a background thread. Validation problems
    /// (nothing selected, unknown device) fail immediately without a thread.
    fn spawn_connect(&mut self, config: ConnConfig) {
        type Work = Box<dyn FnOnce() -> ConnectResult + Send>;
        let work: Work = match &config {
            ConnConfig::Hardware { device, pps } => {
                if device.is_empty() {
                    self.set_warning("no device selected — press Refresh Devices and pick one");
                    self.conn = Connection::Failed { config };
                    return;
                }
                let Some(entry) = self.devices.iter().find(|e| e.token == *device) else {
                    self.set_warning(&format!(
                        "device '{device}' not found — press Refresh Devices"
                    ));
                    self.conn = Connection::Failed { config };
                    return;
                };
                let id = entry.id.clone();
                let pps = *pps;
                Box::new(move || {
                    DacBackend::connect(&id, pps).map(|b| Box::new(b) as Box<dyn LaserBackend>)
                })
            }
            ConnConfig::Ponk {
                address,
                sender_name,
            } => {
                let address = address.clone();
                let sender_name = sender_name.clone();
                Box::new(move || {
                    PonkBackend::connect(&address, &sender_name)
                        .map(|b| Box::new(b) as Box<dyn LaserBackend>)
                })
            }
        };

        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let _ = tx.send(work());
        });
        self.set_info("connecting…");
        self.conn = Connection::Connecting { rx, config };
    }

    /// Drive the connection state machine from the current params. All
    /// transitions happen here, once per cook at most.
    fn reconcile(&mut self) {
        let want = self.params.active;
        let config = self.conn_config();

        // The retry ticket exists only to revive a Failed connection; drop
        // it once the connection is healthy so a Refresh pulsed on a live
        // session can't trigger a surprise reconnect after a later failure.
        if matches!(self.conn, Connection::Connected { .. }) {
            self.force_reconnect = false;
        }

        if !want {
            // Deactivation clears state and status ONCE (edge-triggered):
            // clearing every inactive cook would erase scan warnings the
            // instant poll_devices() raises them.
            if !matches!(self.conn, Connection::Idle) {
                if let Connection::Connected { backend, .. } =
                    std::mem::replace(&mut self.conn, Connection::Idle)
                {
                    Self::teardown(backend);
                }
                self.conn = Connection::Idle;
                self.set_info("");
                self.set_warning("");
            }
            return;
        }

        // Config changed under a live or in-flight connection: retire it.
        let stale = match &self.conn {
            Connection::Connected { config: c, .. } | Connection::Connecting { config: c, .. } => {
                *c != config
            }
            _ => false,
        };
        if stale {
            match std::mem::replace(&mut self.conn, Connection::Idle) {
                Connection::Connected { backend, .. } => Self::teardown(backend),
                // The in-flight connect may already own the device; hold in
                // Draining until it resolves rather than racing a second
                // connect against it.
                Connection::Connecting { rx, .. } => self.conn = Connection::Draining { rx },
                _ => unreachable!(),
            }
            self.set_info("");
        }

        // Wait out an abandoned connect before starting a new one.
        if matches!(self.conn, Connection::Draining { .. }) {
            let resolved = match &self.conn {
                Connection::Draining { rx } => match rx.try_recv() {
                    Err(mpsc::TryRecvError::Empty) => None,
                    Ok(res) => Some(res.ok()),
                    Err(mpsc::TryRecvError::Disconnected) => Some(None),
                },
                _ => unreachable!(),
            };
            match resolved {
                None => return, // still draining
                Some(backend) => {
                    if let Some(backend) = backend {
                        Self::teardown(backend);
                    }
                    self.conn = Connection::Idle;
                }
            }
        }

        // Poll an in-flight connect.
        let outcome = match &self.conn {
            Connection::Connecting { rx, .. } => match rx.try_recv() {
                Ok(res) => Some(res),
                Err(mpsc::TryRecvError::Empty) => return, // still connecting
                Err(mpsc::TryRecvError::Disconnected) => {
                    Some(Err("connect thread terminated unexpectedly".to_string()))
                }
            },
            _ => None,
        };
        if let Some(outcome) = outcome {
            if let Connection::Connecting { config, .. } =
                std::mem::replace(&mut self.conn, Connection::Idle)
            {
                match outcome {
                    Ok(backend) => {
                        self.set_info(&backend.describe());
                        self.set_warning("");
                        self.force_reconnect = false;
                        self.conn = Connection::Connected {
                            backend,
                            config,
                            frames_sent: 0,
                            last_points: 0,
                        };
                    }
                    Err(e) => {
                        self.set_warning(&e);
                        self.set_info("");
                        self.conn = Connection::Failed { config };
                    }
                }
            }
            return;
        }

        let should_connect = match &self.conn {
            Connection::Idle => true,
            Connection::Failed { config: c } => self.force_reconnect || *c != config,
            _ => false,
        };
        if should_connect {
            // Consume the retry ticket only when it actually drives a
            // connect attempt.
            self.force_reconnect = false;
            self.spawn_connect(config);
        }
    }

    /// One cook's data path: map the input channels into points and hand one
    /// frame to the backend. Returns the number of points sent.
    fn stream_frame(
        backend: &mut dyn LaserBackend,
        inputs: &OperatorInputs<ChopInput>,
        params: &LaserDeviceParams,
        points_buf: &mut Vec<Point2>,
    ) -> Result<usize, SendError> {
        let input = match inputs.input(0) {
            Some(input) if input.num_channels() >= 2 => input,
            _ => {
                let _ = backend.blank();
                return Err(SendError::Warn(
                    "input CHOP with at least x and y channels is required".to_string(),
                ));
            }
        };

        let names: Vec<&str> = (0..input.num_channels())
            .map(|i| input.channel_name(i))
            .collect();
        let map = match mapping::resolve_channels(&names) {
            Ok(map) => map,
            Err(e) => {
                let _ = backend.blank();
                return Err(SendError::Warn(e));
            }
        };

        let channels: Vec<&[f32]> = (0..input.num_channels())
            .map(|i| input.channel(i))
            .collect();
        let opts = MapOptions {
            scale: params.scale,
            intensity: params.intensity,
            default_rgb: [
                params.default_color.r,
                params.default_color.g,
                params.default_color.b,
            ],
        };
        mapping::build_points(&channels, input.num_samples(), &map, &opts, points_buf);
        backend.send(points_buf).map(|_| points_buf.len())
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
                address: std::net::SocketAddrV4::new(
                    ponk_protocol::MULTICAST_ADDR.into(),
                    ponk_protocol::DEFAULT_PORT,
                )
                .to_string(),
                sender_name: "TouchDesigner".to_string(),
            },
            conn: Connection::Idle,
            devices: Vec::new(),
            scan_rx: None,
            scanned_once: false,
            force_reconnect: false,
            points_buf: Vec::new(),
            info_msg: String::new(),
            warning_msg: String::new(),
            error_msg: String::new(),
        }
    }
}

impl OpInfo for LaserDeviceChop {
    const OPERATOR_TYPE: &'static str = "Laserdevice";
    const OPERATOR_LABEL: &'static str = "Laser Device";
    const MIN_INPUTS: usize = 1;
    const MAX_INPUTS: usize = 1;
    // Kick off the device scan before the user first opens the Device menu
    // (build_dynamic_menu takes &self, so it cannot scan itself).
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

    // Per-instance status storage — see the field comments on the struct.
    fn set_info(&mut self, info: &str) {
        self.info_msg.replace_range(.., info);
    }

    fn info(&self) -> String {
        self.info_msg.clone()
    }

    fn set_warning(&mut self, warning: &str) {
        self.warning_msg.replace_range(.., warning);
    }

    fn warning(&self) -> String {
        self.warning_msg.clone()
    }

    fn set_error(&mut self, error: &str) {
        self.error_msg.replace_range(.., error);
    }

    fn error(&self) -> String {
        self.error_msg.clone()
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
            self.scanned_once = true;
            self.refresh_devices();
        }
        self.poll_devices();
        self.reconcile();

        // Stream one frame. The outcome is applied after the &mut borrow of
        // self.conn ends (set_warning needs &mut self).
        let outcome = match &mut self.conn {
            Connection::Connected { backend, .. } => Some(Self::stream_frame(
                backend.as_mut(),
                inputs,
                &self.params,
                &mut self.points_buf,
            )),
            _ => None,
        };
        match outcome {
            None => {}
            Some(Ok(n)) => {
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
            // Transient/input-caused: warn but stay connected — the next
            // frame may succeed (e.g. the DAC is auto-reconnecting, or one
            // frame had bad data).
            Some(Err(SendError::Warn(e))) => self.set_warning(&e),
            // The backend is dead: drop to Failed so a config change,
            // Refresh, or completed scan retries.
            Some(Err(SendError::Fatal(e))) => {
                self.set_warning(&e);
                self.set_info("");
                if let Connection::Connected {
                    backend, config, ..
                } = std::mem::replace(&mut self.conn, Connection::Idle)
                {
                    Self::teardown(backend);
                    self.conn = Connection::Failed { config };
                }
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
