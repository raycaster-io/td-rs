//! Output backends: laser DAC hardware (via `laser-dac`) and PONK network
//! output (via `ponk-protocol`), behind one small trait so `lib.rs` treats
//! them uniformly.

use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};

use laser_dac::{Frame, FrameSession, FrameSessionConfig, LaserPoint, ReconnectConfig};
use ponk_protocol::{encode_datagrams, DataFormat, PonkFrame, PonkPath, PonkPoint};

use crate::mapping::Point2;

/// Number of blanked points sent as a safety blank before teardown.
const BLANK_POINTS: usize = 32;

/// Maximum UDP datagram size for PONK output. Kept under a typical ethernet
/// MTU so frames survive networks that drop fragmented UDP.
const PONK_MAX_DATAGRAM: usize = 1400;

/// A destination laser points can be streamed to once per cook.
pub trait LaserBackend: Send {
    /// Submit one frame of points. Must not block the cook thread.
    fn send(&mut self, points: &[Point2]) -> Result<(), String>;
    /// Emit an all-black frame (safety blank), used before teardown and when
    /// the input goes away.
    fn blank(&mut self) -> Result<(), String>;
    /// Human-readable description of the connected output for the info popup.
    fn describe(&self) -> String;
    /// Whether the underlying output still looks healthy.
    fn is_connected(&self) -> bool;
}

/// A discovered DAC, with a TouchDesigner-safe menu token.
#[derive(Clone, Debug)]
pub struct DeviceEntry {
    /// Sanitized token stored by the Device menu parameter.
    pub token: String,
    /// Human-readable label shown in the menu.
    pub label: String,
    /// The laser-dac device id used to open the device.
    pub id: String,
}

fn menu_token(id: &str) -> String {
    let mut token = String::with_capacity(id.len());
    for ch in id.chars() {
        if ch.is_ascii_alphanumeric() {
            token.push(ch.to_ascii_lowercase());
        } else if !token.ends_with('_') {
            token.push('_');
        }
    }
    token.trim_matches('_').to_string()
}

/// Scan for DACs. Blocking (network/USB discovery) — call only from user
/// actions (Refresh pulse) or the first cook, never every cook.
pub fn discover() -> Result<Vec<DeviceEntry>, String> {
    let infos = laser_dac::list_devices().map_err(|e| format!("DAC discovery failed: {e}"))?;
    let mut entries: Vec<DeviceEntry> = Vec::with_capacity(infos.len());
    for info in infos {
        let mut token = menu_token(&info.id);
        let mut n = 1;
        while entries.iter().any(|e| e.token == token) {
            n += 1;
            token = format!("{}_{n}", menu_token(&info.id));
        }
        entries.push(DeviceEntry {
            token,
            label: format!("{} ({:?})", info.name, info.kind),
            id: info.id,
        });
    }
    Ok(entries)
}

/// Hardware output through a laser-dac frame session.
///
/// The session owns a scheduler thread that paces points to the DAC and
/// auto-loops the latest frame, so `send` is a non-blocking latest-wins
/// handoff.
pub struct DacBackend {
    session: FrameSession,
    name: String,
    id: String,
}

impl DacBackend {
    /// Open a device and start an armed frame session. Blocking — call only
    /// on state transitions, not per cook.
    pub fn connect(id: &str, pps: u32) -> Result<Self, String> {
        let dac = laser_dac::open_device(id).map_err(|e| format!("open '{id}' failed: {e}"))?;
        let config = FrameSessionConfig::new(pps).with_reconnect(ReconnectConfig::new());
        let (session, info) = dac
            .start_frame_session(config)
            .map_err(|e| format!("session on '{id}' failed ({pps} pps): {e}"))?;
        session
            .control()
            .arm()
            .map_err(|e| format!("arming '{id}' failed: {e}"))?;
        Ok(Self {
            session,
            name: info.name,
            id: id.to_string(),
        })
    }
}

impl LaserBackend for DacBackend {
    fn send(&mut self, points: &[Point2]) -> Result<(), String> {
        if self.session.is_finished() {
            return Err(format!("session on '{}' has stopped", self.id));
        }
        let points: Vec<LaserPoint> = points
            .iter()
            .map(|p| LaserPoint::new(p.x, p.y, p.r, p.g, p.b, p.i))
            .collect();
        self.session.send_frame(Frame::new(points));
        Ok(())
    }

    fn blank(&mut self) -> Result<(), String> {
        self.send(&[Point2::default(); BLANK_POINTS])
    }

    fn describe(&self) -> String {
        format!("{} ({})", self.name, self.id)
    }

    fn is_connected(&self) -> bool {
        !self.session.is_finished() && self.session.metrics().connected()
    }
}

impl Drop for DacBackend {
    fn drop(&mut self) {
        let _ = self.session.control().stop();
    }
}

/// PONK network output: encodes each frame as one path and sends it as UDP
/// datagrams (multicast by convention, unicast works too).
pub struct PonkBackend {
    socket: UdpSocket,
    dest: SocketAddr,
    sender_id: u32,
    sender_name: String,
    frame_number: u8,
}

impl PonkBackend {
    /// Bind a socket for the destination address. Cheap and non-blocking.
    pub fn connect(address: &str, sender_name: &str) -> Result<Self, String> {
        let dest = address
            .to_socket_addrs()
            .map_err(|e| format!("invalid PONK address '{address}': {e}"))?
            .next()
            .ok_or_else(|| format!("PONK address '{address}' resolved to nothing"))?;
        let bind_addr = if dest.is_ipv4() {
            "0.0.0.0:0"
        } else {
            "[::]:0"
        };
        let socket =
            UdpSocket::bind(bind_addr).map_err(|e| format!("UDP socket bind failed: {e}"))?;
        if let SocketAddr::V4(v4) = dest {
            if v4.ip().is_multicast() {
                socket
                    .set_multicast_ttl_v4(1)
                    .map_err(|e| format!("setting multicast TTL failed: {e}"))?;
            }
        }
        socket
            .set_nonblocking(true)
            .map_err(|e| format!("setting socket non-blocking failed: {e}"))?;
        Ok(Self {
            socket,
            dest,
            sender_id: std::process::id(),
            sender_name: sender_name.to_string(),
            frame_number: 0,
        })
    }

    fn send_frame(&mut self, frame: &PonkFrame) -> Result<(), String> {
        let datagrams = encode_datagrams(frame, DataFormat::XyF32RgbU8, PONK_MAX_DATAGRAM)
            .map_err(|e| format!("PONK encoding failed: {e:?}"))?;
        for datagram in datagrams {
            match self.socket.send_to(&datagram, self.dest) {
                Ok(_) => {}
                // A non-blocking socket with a full send buffer drops the
                // frame; the next cook sends a fresh one.
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(()),
                Err(e) => return Err(format!("PONK send to {} failed: {e}", self.dest)),
            }
        }
        Ok(())
    }
}

/// Build a PONK frame from mapped points. PONK has no intensity field, so
/// each point's intensity modulates its color.
pub fn ponk_frame(
    sender_id: u32,
    sender_name: &str,
    frame_number: u8,
    points: &[Point2],
) -> PonkFrame {
    let to_u8 = |c: u16, i: u16| -> u8 { ((c as u32 * i as u32 / 65535) >> 8) as u8 };
    let path = PonkPath {
        metadata: Vec::new(),
        points: points
            .iter()
            .map(|p| PonkPoint {
                x: p.x,
                y: p.y,
                rgb: [to_u8(p.r, p.i), to_u8(p.g, p.i), to_u8(p.b, p.i)],
            })
            .collect(),
    };
    PonkFrame {
        sender_id,
        sender_name: sender_name.to_string(),
        frame_number,
        paths: if path.points.is_empty() {
            Vec::new()
        } else {
            vec![path]
        },
    }
}

impl LaserBackend for PonkBackend {
    fn send(&mut self, points: &[Point2]) -> Result<(), String> {
        self.frame_number = self.frame_number.wrapping_add(1);
        let frame = ponk_frame(self.sender_id, &self.sender_name, self.frame_number, points);
        self.send_frame(&frame)
    }

    fn blank(&mut self) -> Result<(), String> {
        // A pathless frame tells receivers to draw nothing.
        self.send(&[])
    }

    fn describe(&self) -> String {
        format!("PONK → {} as \"{}\"", self.dest, self.sender_name)
    }

    fn is_connected(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ponk_protocol::PonkAssembler;

    fn points() -> Vec<Point2> {
        (0..100u16)
            .map(|n| Point2 {
                x: (n as f32 / 50.0) - 1.0,
                y: 1.0 - (n as f32 / 50.0),
                r: n * 655,
                g: 65535 - n * 655,
                b: 32767,
                i: 65535,
            })
            .collect()
    }

    #[test]
    fn ponk_frame_roundtrips_through_encode_and_reassembly() {
        let frame = ponk_frame(42, "laser-device test", 7, &points());
        // Force multipart so the reassembler path is exercised.
        let datagrams = encode_datagrams(&frame, DataFormat::XyF32RgbU8, 200).unwrap();
        assert!(datagrams.len() > 1);

        let peer: SocketAddr = "127.0.0.1:5583".parse().unwrap();
        let mut assembler = PonkAssembler::new();
        let mut decoded = None;
        for datagram in &datagrams {
            if let Some(frame) = assembler.push_datagram(datagram, peer).unwrap() {
                decoded = Some(frame);
            }
        }
        assert_eq!(decoded.expect("frame should reassemble"), frame);
    }

    #[test]
    fn ponk_intensity_modulates_color() {
        let point = Point2 {
            x: 0.0,
            y: 0.0,
            r: 65535,
            g: 65535,
            b: 65535,
            i: 32767,
        };
        let frame = ponk_frame(1, "t", 0, &[point]);
        let rgb = frame.paths[0].points[0].rgb;
        assert!(rgb[0] >= 126 && rgb[0] <= 128, "got {}", rgb[0]);
    }

    #[test]
    fn blank_frame_draws_no_points() {
        let frame = ponk_frame(1, "t", 0, &[]);
        assert!(frame.paths.is_empty());
        // Header-only datagram still encodes and decodes. (The decoder may
        // materialize an empty path; what matters is that nothing is drawn.)
        let datagrams = encode_datagrams(&frame, DataFormat::XyF32RgbU8, 1400).unwrap();
        assert_eq!(datagrams.len(), 1);
        let decoded = ponk_protocol::decode_datagram(&datagrams[0])
            .unwrap()
            .unwrap();
        assert_eq!(decoded.sender_id, frame.sender_id);
        assert_eq!(decoded.frame_number, frame.frame_number);
        assert!(decoded.paths.iter().all(|p| p.points.is_empty()));
    }

    #[test]
    fn menu_tokens_are_sanitized() {
        assert_eq!(menu_token("etherdream:aa:bb:cc"), "etherdream_aa_bb_cc");
        assert_eq!(menu_token("idn:host.local"), "idn_host_local");
        assert_eq!(menu_token("__weird--id__"), "weird_id");
    }
}
