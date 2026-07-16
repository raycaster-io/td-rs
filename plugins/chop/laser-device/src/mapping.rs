//! Pure CHOP-channel to laser-point mapping.
//!
//! Everything here operates on plain slices so it can be unit tested without
//! TouchDesigner or any DAC hardware.

/// Plugin-owned laser point. Backends convert this to their own wire types
/// (`laser_dac::LaserPoint`, `ponk_protocol::PonkPoint`).
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Point2 {
    /// X coordinate, -1.0 (left) to 1.0 (right).
    pub x: f32,
    /// Y coordinate, -1.0 (bottom) to 1.0 (top).
    pub y: f32,
    /// Red, 0-65535.
    pub r: u16,
    /// Green, 0-65535.
    pub g: u16,
    /// Blue, 0-65535.
    pub b: u16,
    /// Intensity, 0-65535. Master intensity is folded in here; backends
    /// without a dedicated intensity field modulate color by it instead.
    pub i: u16,
}

/// Input channel indices for each laser point field.
#[derive(Clone, Debug, PartialEq)]
pub struct ChannelMap {
    pub x: usize,
    pub y: usize,
    pub r: Option<usize>,
    pub g: Option<usize>,
    pub b: Option<usize>,
    pub i: Option<usize>,
}

impl ChannelMap {
    pub fn has_color(&self) -> bool {
        self.r.is_some() || self.g.is_some() || self.b.is_some()
    }
}

/// Resolve input channels by name, falling back to positional order.
///
/// Name matching is case-insensitive: `x`/`tx`, `y`/`ty`, `r`/`red`,
/// `g`/`green`, `b`/`blue`, `i`/`intensity`. This matches the channel layout
/// TouchDesigner's native laser workflow uses (x/y/r/g/b, one sample per
/// point). When neither x nor y is named, channels are assigned positionally:
/// 0=x, 1=y, 2=r, 3=g, 4=b, 5=i. Naming exactly one of x/y is an error —
/// silently mapping a channel named `y` onto the X axis positionally would
/// draw a transposed figure with no warning.
pub fn resolve_channels(names: &[&str]) -> Result<ChannelMap, String> {
    if names.len() < 2 {
        return Err(format!(
            "input must have at least 2 channels (x, y), got {}",
            names.len()
        ));
    }

    let find = |aliases: &[&str]| -> Option<usize> {
        names
            .iter()
            .position(|n| aliases.contains(&n.trim().to_ascii_lowercase().as_str()))
    };

    let x = find(&["x", "tx"]);
    let y = find(&["y", "ty"]);
    match (x, y) {
        (Some(x), Some(y)) => Ok(ChannelMap {
            x,
            y,
            r: find(&["r", "red"]),
            g: find(&["g", "green"]),
            b: find(&["b", "blue"]),
            i: find(&["i", "intensity"]),
        }),
        (None, None) => {
            let nth = |n: usize| -> Option<usize> { (names.len() > n).then_some(n) };
            Ok(ChannelMap {
                x: 0,
                y: 1,
                r: nth(2),
                g: nth(3),
                b: nth(4),
                i: nth(5),
            })
        }
        (Some(_), None) => Err("input names an x channel but no y — name both \
             (x/y) or neither (positional order)"
            .to_string()),
        (None, Some(_)) => Err("input names a y channel but no x — name both \
             (x/y) or neither (positional order)"
            .to_string()),
    }
}

/// Options applied while building points.
#[derive(Clone, Copy, Debug)]
pub struct MapOptions {
    /// Multiplied into x/y before clamping to [-1, 1].
    pub scale: f32,
    /// Master intensity, 0-1. Folded into the intensity field so hardware
    /// modulates output without losing color resolution.
    pub intensity: f32,
    /// Color used when the input has no color channels at all.
    pub default_rgb: [f32; 3],
}

fn color_to_u16(v: f32) -> u16 {
    (v.clamp(0.0, 1.0) * 65535.0) as u16
}

/// Build laser points from input channel data.
///
/// `channels[c]` is channel `c`'s sample slice; every slice must hold at
/// least `num_samples` values. `out` is cleared and refilled so the buffer
/// can be reused across cooks.
///
/// Rules:
/// - x/y are scaled then hard-clamped to [-1, 1] (never let a bad input
///   drive a scanner out of range). `f32::clamp` passes NaN through, so
///   non-finite coordinates are handled first: the point is emitted fully
///   blanked (beam off) at center rather than sending NaN to a scanner.
/// - Colors are clamped to [0, 1] then scaled to 16 bit. If the input has no
///   color channels every point gets `default_rgb`; if it has some, missing
///   channels are 0.
/// - The intensity field is the per-point `i` channel (default full) times
///   the master intensity.
pub fn build_points(
    channels: &[&[f32]],
    num_samples: usize,
    map: &ChannelMap,
    opts: &MapOptions,
    out: &mut Vec<Point2>,
) {
    out.clear();
    out.reserve(num_samples);

    let has_color = map.has_color();
    let master = opts.intensity.clamp(0.0, 1.0);
    let color = |idx: Option<usize>, s: usize| -> u16 {
        match idx {
            Some(c) => color_to_u16(channels[c][s]),
            None => 0,
        }
    };

    for s in 0..num_samples {
        let x = channels[map.x][s] * opts.scale;
        let y = channels[map.y][s] * opts.scale;
        if !x.is_finite() || !y.is_finite() {
            // NaN survives f32::clamp, and PONK's encoder rejects non-finite
            // coordinates outright. Emit a blanked (beam-off) point instead.
            out.push(Point2::default());
            continue;
        }

        let (r, g, b) = if has_color {
            (color(map.r, s), color(map.g, s), color(map.b, s))
        } else {
            (
                color_to_u16(opts.default_rgb[0]),
                color_to_u16(opts.default_rgb[1]),
                color_to_u16(opts.default_rgb[2]),
            )
        };
        let i = match map.i {
            Some(c) => channels[c][s].clamp(0.0, 1.0) * master,
            None => master,
        };
        out.push(Point2 {
            x: x.clamp(-1.0, 1.0),
            y: y.clamp(-1.0, 1.0),
            r,
            g,
            b,
            i: color_to_u16(i),
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const OPTS: MapOptions = MapOptions {
        scale: 1.0,
        intensity: 1.0,
        default_rgb: [1.0, 1.0, 1.0],
    };

    #[test]
    fn resolves_by_name_case_insensitive() {
        let map = resolve_channels(&["B", " X ", "Intensity", "y", "G", "r"]).unwrap();
        assert_eq!(map.x, 1);
        assert_eq!(map.y, 3);
        assert_eq!(map.r, Some(5));
        assert_eq!(map.g, Some(4));
        assert_eq!(map.b, Some(0));
        assert_eq!(map.i, Some(2));
    }

    #[test]
    fn resolves_tx_ty_aliases() {
        let map = resolve_channels(&["tx", "ty", "tz"]).unwrap();
        assert_eq!(map.x, 0);
        assert_eq!(map.y, 1);
        assert!(!map.has_color());
    }

    #[test]
    fn falls_back_to_positional_order() {
        let map = resolve_channels(&["chan1", "chan2", "chan3"]).unwrap();
        assert_eq!(map.x, 0);
        assert_eq!(map.y, 1);
        assert_eq!(map.r, Some(2));
        assert_eq!(map.g, None);
        assert_eq!(map.b, None);
        assert_eq!(map.i, None);
    }

    #[test]
    fn rejects_fewer_than_two_channels() {
        assert!(resolve_channels(&["x"]).is_err());
        assert!(resolve_channels(&[]).is_err());
    }

    #[test]
    fn rejects_partial_xy_naming() {
        // A channel explicitly named y must never drive the X axis via the
        // positional fallback.
        assert!(resolve_channels(&["y", "brightness"]).is_err());
        assert!(resolve_channels(&["x", "brightness"]).is_err());
        assert!(resolve_channels(&["a", "b", "x", "d"]).is_err());
    }

    #[test]
    fn non_finite_coordinates_emit_blanked_points() {
        let map = resolve_channels(&["x", "y"]).unwrap();
        let mut out = Vec::new();
        build_points(
            &[&[f32::NAN, 0.5, f32::INFINITY], &[0.0, 0.5, 0.0]],
            3,
            &map,
            &OPTS,
            &mut out,
        );
        assert_eq!(out[0], Point2::default()); // NaN x → blanked
        assert_eq!(out[1].x, 0.5); // finite point untouched
        assert_eq!(out[1].i, 65535);
        assert_eq!(out[2], Point2::default()); // inf x → blanked
        assert!(out.iter().all(|p| p.x.is_finite() && p.y.is_finite()));
    }

    #[test]
    fn non_finite_via_scale_emits_blanked_points() {
        let map = resolve_channels(&["x", "y"]).unwrap();
        let mut out = Vec::new();
        // 0.0 * inf = NaN happens inside the scale multiply too.
        let opts = MapOptions {
            scale: f32::INFINITY,
            ..OPTS
        };
        build_points(&[&[0.0], &[1.0]], 1, &map, &opts, &mut out);
        assert_eq!(out[0], Point2::default());
    }

    #[test]
    fn no_color_channels_uses_default_color() {
        let map = resolve_channels(&["x", "y"]).unwrap();
        let mut out = Vec::new();
        let opts = MapOptions {
            default_rgb: [1.0, 0.5, 0.0],
            ..OPTS
        };
        build_points(&[&[0.25], &[-0.5]], 1, &map, &opts, &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].x, 0.25);
        assert_eq!(out[0].y, -0.5);
        assert_eq!(out[0].r, 65535);
        assert_eq!(out[0].g, 32767);
        assert_eq!(out[0].b, 0);
        assert_eq!(out[0].i, 65535);
    }

    #[test]
    fn partial_color_channels_zero_missing_ones() {
        let map = resolve_channels(&["x", "y", "r"]).unwrap();
        let mut out = Vec::new();
        build_points(&[&[0.0], &[0.0], &[1.0]], 1, &map, &OPTS, &mut out);
        assert_eq!(out[0].r, 65535);
        assert_eq!(out[0].g, 0);
        assert_eq!(out[0].b, 0);
    }

    #[test]
    fn scale_applies_before_clamp() {
        let map = resolve_channels(&["x", "y"]).unwrap();
        let mut out = Vec::new();
        let opts = MapOptions { scale: 2.0, ..OPTS };
        build_points(&[&[0.25, 0.9], &[-0.25, -0.9]], 2, &map, &opts, &mut out);
        assert_eq!(out[0].x, 0.5);
        assert_eq!(out[0].y, -0.5);
        assert_eq!(out[1].x, 1.0); // 1.8 clamped
        assert_eq!(out[1].y, -1.0); // -1.8 clamped
    }

    #[test]
    fn colors_clamp_to_unit_range() {
        let map = resolve_channels(&["x", "y", "r", "g", "b"]).unwrap();
        let mut out = Vec::new();
        build_points(
            &[&[0.0], &[0.0], &[2.0], &[-1.0], &[0.5]],
            1,
            &map,
            &OPTS,
            &mut out,
        );
        assert_eq!(out[0].r, 65535);
        assert_eq!(out[0].g, 0);
        assert_eq!(out[0].b, 32767);
    }

    #[test]
    fn master_intensity_scales_intensity_channel() {
        let map = resolve_channels(&["x", "y", "i"]).unwrap();
        // "i" resolves as intensity, not as a positional color channel.
        assert_eq!(map.i, Some(2));
        let mut out = Vec::new();
        let opts = MapOptions {
            intensity: 0.5,
            ..OPTS
        };
        build_points(&[&[0.0], &[0.0], &[0.5]], 1, &map, &opts, &mut out);
        assert_eq!(out[0].i, (0.25f32 * 65535.0) as u16);
        // Colors keep full resolution; brightness rides the intensity field.
        assert_eq!(out[0].r, 65535);
    }

    #[test]
    fn output_buffer_is_reused() {
        let map = resolve_channels(&["x", "y"]).unwrap();
        let mut out = Vec::new();
        build_points(
            &[&[0.0, 0.1, 0.2], &[0.0, 0.1, 0.2]],
            3,
            &map,
            &OPTS,
            &mut out,
        );
        assert_eq!(out.len(), 3);
        build_points(&[&[0.5], &[0.5]], 1, &map, &OPTS, &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].x, 0.5);
    }
}
