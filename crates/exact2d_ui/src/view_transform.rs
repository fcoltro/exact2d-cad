//! World ↔ screen mapping for the drawing canvas (spec §6.1 viewport).

/// Maps world coordinates (y-up) to canvas pixels (y-down, origin top-left).
#[derive(Clone, Debug)]
pub struct ViewTransform {
    /// World point at the center of the canvas.
    pub center: (f64, f64),
    /// Pixels per world unit.
    pub zoom: f64,
    /// Canvas size in pixels.
    pub width: f64,
    pub height: f64,
    /// Smallest amount of the drawing (in coordinate units) allowed to fill the
    /// viewport — the zoom-IN limit (keeps f32 rendering precise).
    pub min_visible: f64,
    /// Largest amount of the drawing allowed to fill the viewport — the zoom-OUT
    /// limit (avoids large-coordinate clipping).
    pub max_visible: f64,
}

impl ViewTransform {
    pub fn new(width: f64, height: f64) -> Self {
        ViewTransform {
            center: (0.0, 0.0), zoom: 50.0, width, height,
            min_visible: 0.05, max_visible: 50_000.0, // mm defaults
        }
    }

    /// Set the visible-width limits (from the active unit) and re-clamp the zoom.
    pub fn set_visible_range(&mut self, min_visible: f64, max_visible: f64) {
        self.min_visible = min_visible.max(1e-12);
        self.max_visible = max_visible.max(self.min_visible * 2.0);
        self.zoom = self.clamp_zoom(self.zoom);
    }

    /// Clamp a zoom value to the unit's visible range, given the current viewport.
    fn clamp_zoom(&self, zoom: f64) -> f64 {
        let dim = self.width.max(self.height).max(1.0);
        let zoom_min = dim / self.max_visible; // most zoomed out
        let zoom_max = dim / self.min_visible; // most zoomed in
        zoom.clamp(zoom_min, zoom_max)
    }

    pub fn world_to_screen(&self, wx: f64, wy: f64) -> (f64, f64) {
        let sx = (wx - self.center.0) * self.zoom + self.width / 2.0;
        let sy = (self.center.1 - wy) * self.zoom + self.height / 2.0;
        (sx, sy)
    }

    pub fn screen_to_world(&self, sx: f64, sy: f64) -> (f64, f64) {
        let wx = self.center.0 + (sx - self.width / 2.0) / self.zoom;
        let wy = self.center.1 - (sy - self.height / 2.0) / self.zoom;
        (wx, wy)
    }

    /// World size of one screen pixel (drives snap tolerance + grid spacing).
    pub fn pixel_world_size(&self) -> f64 { 1.0 / self.zoom }

    /// Pan by a screen-space delta (pixels).
    pub fn pan_pixels(&mut self, dx: f64, dy: f64) {
        self.center.0 -= dx / self.zoom;
        self.center.1 += dy / self.zoom;
    }

    /// Zoom by `factor`, keeping world point `(wx, wy)` fixed under the cursor.
    /// The zoom is clamped to the active unit's safe range, and the anchor is only
    /// shifted by the zoom change that actually took effect.
    pub fn zoom_at(&mut self, wx: f64, wy: f64, factor: f64) {
        let old = self.zoom;
        let new = self.clamp_zoom(old * factor);
        if new == old { return; } // already at a limit — don't drift the centre
        let eff = new / old; // effective factor after clamping
        self.zoom = new;
        self.center.0 = wx + (self.center.0 - wx) / eff;
        self.center.1 = wy + (self.center.1 - wy) / eff;
    }

    /// Whether the zoom is at its in/out limit (for UI feedback).
    pub fn at_zoom_in_limit(&self) -> bool {
        self.zoom >= self.clamp_zoom(self.zoom * 1.0001)
    }
    pub fn at_zoom_out_limit(&self) -> bool {
        self.zoom <= self.clamp_zoom(self.zoom * 0.9999)
    }

    /// ZOOM EXTENTS: frame a world bounding box with margin (clamped to the range).
    pub fn zoom_to_bounds(&mut self, x0: f64, y0: f64, x1: f64, y1: f64) {
        let w = (x1 - x0).max(1e-9);
        let h = (y1 - y0).max(1e-9);
        let margin = 1.1;
        let zx = self.width / (w * margin);
        let zy = self.height / (h * margin);
        self.zoom = self.clamp_zoom(zx.min(zy));
        self.center = ((x0 + x1) / 2.0, (y0 + y1) / 2.0);
    }

    /// Visible world rectangle (xmin, ymin, xmax, ymax).
    pub fn visible_bounds(&self) -> (f64, f64, f64, f64) {
        let hw = self.width / (2.0 * self.zoom);
        let hh = self.height / (2.0 * self.zoom);
        (self.center.0 - hw, self.center.1 - hh, self.center.0 + hw, self.center.1 + hh)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let v = ViewTransform::new(800.0, 600.0);
        let (sx, sy) = v.world_to_screen(3.5, -2.0);
        let (wx, wy) = v.screen_to_world(sx, sy);
        assert!((wx - 3.5).abs() < 1e-9 && (wy + 2.0).abs() < 1e-9);
    }

    #[test]
    fn center_is_screen_center() {
        let v = ViewTransform::new(800.0, 600.0);
        let (sx, sy) = v.world_to_screen(0.0, 0.0);
        assert!((sx - 400.0).abs() < 1e-9 && (sy - 300.0).abs() < 1e-9);
    }

    #[test]
    fn zoom_at_keeps_anchor() {
        let mut v = ViewTransform::new(800.0, 600.0);
        let before = v.world_to_screen(2.0, 1.0);
        v.zoom_at(2.0, 1.0, 2.5);
        let after = v.world_to_screen(2.0, 1.0);
        assert!((before.0 - after.0).abs() < 1e-6 && (before.1 - after.1).abs() < 1e-6);
    }

    #[test]
    fn zoom_is_clamped_to_unit_range() {
        let mut v = ViewTransform::new(800.0, 600.0);
        v.set_visible_range(0.05, 50_000.0); // mm
        let dim = 800.0_f64; // max(width, height)
        // Zoom in absurdly far → capped at the min-visible limit.
        for _ in 0..200 { v.zoom_at(0.0, 0.0, 2.0); }
        assert!((v.zoom - dim / 0.05).abs() / (dim / 0.05) < 1e-9, "zoom-in not capped: {}", v.zoom);
        assert!(v.at_zoom_in_limit());
        // Zoom out absurdly far → capped at the max-visible limit.
        for _ in 0..400 { v.zoom_at(0.0, 0.0, 0.5); }
        assert!((v.zoom - dim / 50_000.0).abs() / (dim / 50_000.0) < 1e-9, "zoom-out not capped: {}", v.zoom);
        assert!(v.at_zoom_out_limit());
    }

    #[test]
    fn zoom_to_bounds_frames_box() {
        let mut v = ViewTransform::new(800.0, 600.0);
        v.zoom_to_bounds(0.0, 0.0, 100.0, 50.0);
        assert_eq!(v.center, (50.0, 25.0));
        // The box must fit within the visible area.
        let (x0, y0, x1, y1) = v.visible_bounds();
        assert!(x0 <= 0.0 && x1 >= 100.0 && y0 <= 0.0 && y1 >= 50.0);
    }
}
