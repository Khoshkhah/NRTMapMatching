//! The map matching of NRTMapMatching in plain Rust.
//!
//! Each function is a line-by-line port of the Python code in `sources/`. The float operations
//! are in the same sequence as in Python, so that the two engines give the same numbers. Where
//! the Python code has a quirk (for example it mixes DataFrame labels and positions), the port
//! keeps the quirk: the results must stay equal.

use rstar::primitives::{GeomWithData, Rectangle};
use rstar::{RTree, AABB};
use std::sync::Arc;

pub type P = [f64; 2];

/// `geotools.distance2d`: `(dx ** 2 + dy ** 2) ** 0.5`. `powf(0.5)` calls the same C `pow` as Python.
#[inline]
pub fn dist(a: P, b: P) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    (dx * dx + dy * dy).powf(0.5)
}

/// `geotools.calculate_bearing_angle`.
#[inline]
pub fn bearing(a: P, b: P) -> f64 {
    let d = (b[0] - a[0]).atan2(b[1] - a[1]).to_degrees();
    if d < 0.0 {
        d + 360.0
    } else {
        d
    }
}

/// `geotools.polyLength`.
pub fn poly_len(s: &[P]) -> f64 {
    let mut l = 0.0;
    for w in s.windows(2) {
        l += dist(w[0], w[1]);
    }
    l
}

/// `geotools.lineOffsetWithMinimumDistanceToPoint`: (offset, distance).
fn line_offset(p: P, p1: P, p2: P) -> (f64, f64) {
    let d = dist(p1, p2);
    let u = ((p[0] - p1[0]) * (p2[0] - p1[0])) + ((p[1] - p1[1]) * (p2[1] - p1[1]));
    if u <= 0.0 {
        (0.0, dist(p, p1))
    } else if u >= d * d {
        (d, dist(p, p2))
    } else {
        let f = u / d;
        let q = [
            ((d - f) * p1[0] + f * p2[0]) / d,
            ((d - f) * p1[1] + f * p2[1]) / d,
        ];
        (f, dist(p, q))
    }
}

/// `geotools.polygonOffsetWithMinimumDistanceToPoint`: (offset, match point).
fn poly_offset(p: P, s: &[P]) -> (f64, P) {
    let mut mi = 0;
    let (mut mo, mut md) = line_offset(p, s[0], s[1]);
    for i in 1..s.len() - 1 {
        let (o, d) = line_offset(p, s[i], s[i + 1]);
        if d < md {
            mi = i;
            mo = o;
            md = d;
        }
    }
    let mut prefix = 0.0;
    for i in 0..mi {
        prefix += dist(s[i], s[i + 1]);
    }
    let d = dist(s[mi], s[mi + 1]);
    let (p1, p2) = (s[mi], s[mi + 1]);
    let pos = [
        (1.0 - mo / d) * p1[0] + (mo / d) * p2[0],
        (1.0 - mo / d) * p1[1] + (mo / d) * p2[1],
    ];
    (mo + prefix, pos)
}

/// `geotools.offsetBearing`.
fn offset_bearing(s: &[P], offset: f64) -> Result<f64, String> {
    if offset > poly_len(s) {
        return Err("offset is greater than polygon length.".into());
    }
    if offset < 0.0 {
        return Err("expected the offset to be a positive number".into());
    }
    let mut sum = 0.0;
    let mut seg = (s[0], s[1]);
    for w in s.windows(2) {
        seg = (w[0], w[1]);
        sum += dist(w[0], w[1]);
        if sum > offset {
            break;
        }
    }
    Ok(bearing(seg.0, seg.1))
}

/// `sumolib.geomhelper.distancePointToPolygon` (not perpendicular): the filter of `Net.getNeighboringEdges`.
fn sumo_dist_to_poly(p: P, s: &[P]) -> f64 {
    let sd = |a: P, b: P| ((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1])).sqrt();
    let mut min = f64::INFINITY;
    for w in s.windows(2) {
        let (p1, p2) = (w[0], w[1]);
        let d = sd(p1, p2);
        let u = (p[0] - p1[0]) * (p2[0] - p1[0]) + (p[1] - p1[1]) * (p2[1] - p1[1]);
        let off = if d == 0.0 || u < 0.0 || u > d * d {
            if u < 0.0 {
                0.0
            } else {
                d
            }
        } else {
            u / d
        };
        let dd = if off == 0.0 {
            sd(p, p1)
        } else {
            let f = off / sd(p1, p2);
            sd(p, [p1[0] + f * (p2[0] - p1[0]), p1[1] + f * (p2[1] - p1[1])])
        };
        if dd < min {
            min = dd;
        }
    }
    min
}

// ---------------------------------------------------------------------------------------------
// GPS cleaning, enrichment and stop detection (cleandata.py)
// ---------------------------------------------------------------------------------------------

/// The rows of a cleaned trajectory (columns x, y, timestamp, speed, bearing, stopindex).
#[derive(Default, Debug, Clone)]
pub struct Rows {
    pub x: Vec<f64>,
    pub y: Vec<f64>,
    pub ts: Vec<i64>,
    pub speed: Vec<f64>,
    pub bearing: Vec<f64>,
    pub stop: Vec<i64>,
}

impl Rows {
    fn push(&mut self, p: P, ts: i64, speed: f64, bearing: f64, stop: i64) {
        self.x.push(p[0]);
        self.y.push(p[1]);
        self.ts.push(ts);
        self.speed.push(speed);
        self.bearing.push(bearing);
        self.stop.push(stop);
    }

    /// `xystop_point_editing`: the median x and y of each stop for all points of that stop.
    fn stop_medians(&mut self) {
        let median = |mut v: Vec<f64>| {
            v.sort_by(|a, b| a.total_cmp(b));
            let n = v.len();
            if n % 2 == 1 {
                v[n / 2]
            } else {
                (v[n / 2 - 1] + v[n / 2]) / 2.0
            }
        };
        let mut i = 0;
        while i < self.stop.len() {
            let s = self.stop[i];
            if s <= 0 {
                i += 1;
                continue;
            }
            // the points of one stop index are consecutive
            let idx: Vec<usize> = (i..self.stop.len()).filter(|&k| self.stop[k] == s).collect();
            let mx = median(idx.iter().map(|&k| self.x[k]).collect());
            let my = median(idx.iter().map(|&k| self.y[k]).collect());
            for &k in &idx {
                self.x[k] = mx;
                self.y[k] = my;
            }
            i = idx.last().unwrap() + 1;
        }
    }
}

/// The stop index of a new row: 0 when it moves, else the number of the stop.
fn stop_index(speed: f64, out: &Rows, stops: &mut i64) -> i64 {
    if speed > 0.0 {
        0
    } else {
        if out.stop.last() == Some(&0) {
            *stops += 1;
        }
        *stops
    }
}

/// `cleaningData` after the column check, the sort, the `groupby("timestamp").last()` and the
/// projection, which stay in Python.
pub fn clean(
    x: &[f64],
    y: &[f64],
    ts: &[i64],
    speed: &[f64],
    brg: &[f64],
    min_speed_for_bearing: f64,
    max_speed_for_outlier: f64,
) -> Rows {
    let mut out = Rows::default();
    let mut cur = 0usize;
    let mut pc = [x[0], y[0]];
    let mut stops = 1i64;
    let mut speed_current = speed[0];
    for i in 1..x.len() {
        let pn = [x[i], y[i]];
        let est = dist(pc, pn) / (ts[i] - ts[cur]) as f64;
        speed_current = speed[cur];
        let mut b = brg[cur];
        if est < max_speed_for_outlier {
            if speed_current < min_speed_for_bearing && !out.x.is_empty() {
                b = *out.bearing.last().unwrap();
            }
            let s = stop_index(speed_current, &out, &mut stops);
            out.push(pc, ts[cur], speed_current, b, s);
            pc = pn;
            cur = i;
        }
    }
    let sp = speed[cur];
    let mut b = brg[cur];
    if sp < min_speed_for_bearing && !out.x.is_empty() {
        b = *out.bearing.last().unwrap();
    }
    // the Python code tests the speed of the last loop step here, not `sp`
    let s = stop_index(speed_current, &out, &mut stops);
    out.push(pc, ts[cur], sp, b, s);
    out.stop_medians();
    out
}

/// `removeOutlier` with its default limit (richdata calls it so).
fn remove_outlier(x: &[f64], y: &[f64], ts: &[i64], max_speed: f64) -> (Vec<P>, Vec<i64>) {
    let mut p = vec![[x[0], y[0]]];
    let mut t = vec![ts[0]];
    for i in 1..x.len() {
        let pc = [x[i], y[i]];
        let s = dist(pc, *p.last().unwrap()) / (ts[i] - *t.last().unwrap()) as f64;
        if s < max_speed {
            p.push(pc);
            t.push(ts[i]);
        }
    }
    (p, t)
}

/// `pandas.Series.ewm(alpha=alpha).mean()` (adjust=True, ignore_na=False), the pandas loop.
fn ewm_mean(v: &[f64], alpha: f64) -> Vec<f64> {
    let f = 1.0 - alpha;
    let mut out = Vec::with_capacity(v.len());
    let mut w = v[0];
    let mut old = 1.0;
    out.push(w);
    for &cur in &v[1..] {
        let obs = !cur.is_nan();
        if !w.is_nan() {
            old *= f;
            if obs {
                if w != cur {
                    w = old * w + 1.0 * cur;
                    w /= old + 1.0;
                }
                old += 1.0;
            }
        } else if obs {
            w = cur;
        }
        out.push(w);
    }
    out
}

/// `smoothingPoint(type="local")`.
fn smooth(p: &[P], ts: &[i64], alpha: f64) -> Vec<P> {
    let n = p.len();
    let mut dt = vec![0f64; n];
    let mut vx = vec![f64::NAN; n];
    let mut vy = vec![f64::NAN; n];
    for i in 0..n {
        if i + 1 < n {
            dt[i] = (ts[i + 1] - ts[i]) as f64;
            vx[i] = (p[i + 1][0] - p[i][0]) / dt[i];
            vy[i] = (p[i + 1][1] - p[i][1]) / dt[i];
        } else {
            dt[i] = 1.0;
        }
    }
    let vx2 = ewm_mean(&vx, alpha);
    let vy2 = ewm_mean(&vy, alpha);
    let mut out = vec![p[0]];
    for i in 0..n - 1 {
        out.push([p[i][0] + dt[i] * vx2[i], p[i][1] + dt[i] * vy2[i]]);
    }
    out
}

/// `richdata` after the projection: outliers, smoothing, speed and bearing, stops.
#[allow(clippy::too_many_arguments)]
pub fn enrich(
    x: &[f64],
    y: &[f64],
    ts: &[i64],
    alpha: f64,
    min_speed: f64,
    max_speed_for_outlier: f64,
    min_speed_for_bearing: f64,
) -> Result<Rows, String> {
    let (p1, t1) = remove_outlier(x, y, ts, 50.0);
    let p = smooth(&p1, &t1, alpha);
    let mut out = Rows::default();
    let mut pc = p[0];
    let mut tc = t1[0];
    let mut stops = 1i64;
    let mut speed_current = f64::NAN;
    for i in 1..p.len() {
        let pn = p[i];
        let tn = t1[i];
        speed_current = dist(pc, pn) / (tn - tc) as f64;
        if speed_current < min_speed {
            speed_current = 0.0;
        }
        let mut b = bearing(pc, pn);
        if speed_current < max_speed_for_outlier {
            if speed_current < min_speed_for_bearing && !out.x.is_empty() {
                b = *out.bearing.last().unwrap();
            }
            let s = stop_index(speed_current, &out, &mut stops);
            out.push(pc, tc, speed_current, b, s);
            pc = pn;
            tc = tn;
        }
    }
    if out.x.is_empty() {
        return Err("list index out of range".into());
    }
    let sp = *out.speed.last().unwrap();
    let b = *out.bearing.last().unwrap();
    let s = stop_index(speed_current, &out, &mut stops);
    out.push(pc, tc, sp, b, s);
    out.stop_medians();
    Ok(out)
}

// ---------------------------------------------------------------------------------------------
// Cubic Bezier interpolation (interpolation.py)
// ---------------------------------------------------------------------------------------------

/// The rows of an interpolated trajectory. `speed` is NaN where Python has `None`.
#[derive(Default, Debug, Clone)]
pub struct Interp {
    /// The index label of the row after `concat(...).reset_index(drop=True).dropna()`.
    pub label: Vec<i64>,
    pub x: Vec<f64>,
    pub y: Vec<f64>,
    pub ts: Vec<i64>,
    pub speed: Vec<f64>,
    pub bearing: Vec<f64>,
    pub origin: Vec<bool>,
    pub stop: Vec<i64>,
}

impl Interp {
    fn push(&mut self, p: P, ts: i64, speed: f64, bearing: f64, origin: bool, stop: i64) {
        self.x.push(p[0]);
        self.y.push(p[1]);
        self.ts.push(ts);
        self.speed.push(speed);
        self.bearing.push(bearing);
        self.origin.push(origin);
        self.stop.push(stop);
    }
}

#[derive(Clone, Copy)]
struct Knot {
    p: P,
    ts: f64,
    speed: f64,
    bearing: f64,
    stop: i64,
}

fn bezier3(p0: P, p1: P, p2: P, p3: P, t: f64) -> P {
    let b0 = (1.0 - t).powf(3.0);
    let b1 = 3.0 * (1.0 - t).powf(2.0) * t;
    let b2 = 3.0 * (1.0 - t) * t.powf(2.0);
    let b3 = t.powf(3.0);
    [
        b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0],
        b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1],
    ]
}

fn bezier2(p0: P, p1: P, p2: P, t: f64) -> P {
    let b0 = (1.0 - t).powf(2.0);
    let b1 = 2.0 * (1.0 - t) * t;
    let b2 = t.powf(2.0);
    [
        b0 * p0[0] + b1 * p1[0] + b2 * p2[0],
        b0 * p0[1] + b1 * p1[1] + b2 * p2[1],
    ]
}

/// `bezierInterpolation(a, b, sample_rate)`, appended to `out`.
fn bezier_segment(a: Knot, b: Knot, sr: f64, out: &mut Interp) {
    let dt = b.ts - a.ts;
    if a.stop > 0 && b.stop > 0 {
        let mut time = a.ts;
        while time <= b.ts {
            out.push(a.p, time as i64, 0.0, a.bearing, time == a.ts || time == b.ts, a.stop);
            time += sr;
            if time > b.ts && time - sr != b.ts {
                time = b.ts;
            }
        }
        return;
    }
    let d = dist(a.p, b.p);
    let va = [(a.speed + 1.0) * (a.bearing * std::f64::consts::PI / 180.0).sin(),
              (a.speed + 1.0) * (a.bearing * std::f64::consts::PI / 180.0).cos()];
    let vb = [(b.speed + 1.0) * (b.bearing * std::f64::consts::PI / 180.0).sin(),
              (b.speed + 1.0) * (b.bearing * std::f64::consts::PI / 180.0).cos()];
    let control = 10.0 + a.speed.max_py(b.speed).powf(1.2);
    let sa = (a.speed + 1.0).sqrt();
    let sb = (b.speed + 1.0).sqrt();
    let alpha = ((0.5 * sa) / (sa + control)) * d * 3.0;
    let beta = ((0.5 * sb) / (sb + control)) * d * 3.0;
    let c0 = [a.p[0] + (alpha * va[0]) / 3.0, a.p[1] + (alpha * va[1]) / 3.0];
    let c1 = [b.p[0] - (beta * vb[0]) / 3.0, b.p[1] - (beta * vb[1]) / 3.0];
    let mut time = 0.0;
    while time + a.ts <= b.ts {
        let t = time / dt;
        let pt = bezier3(a.p, c0, c1, b.p, t);
        let (origin, speed) = if t == 1.0 && t != 0.0 {
            (true, f64::NAN)
        } else {
            let pn = bezier3(a.p, c0, c1, b.p, t + (sr / dt));
            (t == 0.0, dist(pt, pn) / sr)
        };
        let q0 = [3.0 * (c0[0] - a.p[0]), 3.0 * (c0[1] - a.p[1])];
        let q1 = [3.0 * (c1[0] - c0[0]), 3.0 * (c1[1] - c0[1])];
        let q2 = [3.0 * (b.p[0] - c1[0]), 3.0 * (b.p[1] - c1[1])];
        let v = bezier2(q0, q1, q2, t);
        out.push(pt, (time + a.ts) as i64, speed, bearing([0.0, 0.0], v), origin, 0);
        time += sr;
        if time > dt && time - sr != dt {
            time = dt;
        }
    }
}

/// Python's `max(a, b)` for floats: `b` only when `b > a`.
trait MaxPy {
    fn max_py(self, o: f64) -> f64;
}
impl MaxPy for f64 {
    fn max_py(self, o: f64) -> f64 {
        if o > self {
            o
        } else {
            self
        }
    }
}

/// `interpolateTrajectory(traj, sample_rate)`. The input needs 3 rows or more.
#[allow(clippy::too_many_arguments)]
pub fn interpolate(
    x: &[f64],
    y: &[f64],
    ts: &[f64],
    speed: &[f64],
    brg: &[f64],
    stop: &[i64],
    sr: f64,
) -> Interp {
    let k = |i: usize| Knot { p: [x[i], y[i]], ts: ts[i], speed: speed[i], bearing: brg[i], stop: stop[i] };
    let n = x.len();
    let mut all = Interp::default();
    let mut a = k(0);
    for i in 1..n - 1 {
        let b = k(i);
        let start = all.x.len();
        bezier_segment(a, b, sr, &mut all);
        a = b;
        if a.stop > 0 && all.x.len() > start {
            let l = all.x.len() - 1;
            a.p = [all.x[l], all.y[l]];
        }
    }
    if let Some(l) = all.speed.last_mut() {
        *l = speed[n - 1];
    }
    // df.dropna()
    let mut out = Interp::default();
    for i in 0..all.x.len() {
        if [all.x[i], all.y[i], all.speed[i], all.bearing[i]].iter().any(|v| v.is_nan()) {
            continue;
        }
        out.label.push(i as i64);
        out.push([all.x[i], all.y[i]], all.ts[i], all.speed[i], all.bearing[i], all.origin[i], all.stop[i]);
    }
    out
}

// ---------------------------------------------------------------------------------------------
// The network and the R-tree (network.py, the part that the matcher reads)
// ---------------------------------------------------------------------------------------------

/// The road network as the matcher reads it. The edge index is the position in `Net._edgeidlist`.
pub struct Network {
    pub shapes: Vec<Vec<P>>,
    pub speed: Vec<f64>,
    pub from: Vec<usize>,
    pub to: Vec<usize>,
    pub node_out: Vec<Vec<usize>>,
    pub node_in: Vec<Vec<usize>>,
    /// `polyLength` of each edge shape (the road interpolation).
    pub length: Vec<f64>,
    tree: RTree<GeomWithData<Rectangle<P>, usize>>,
}

impl Network {
    pub fn new(
        shapes: Vec<Vec<P>>,
        speed: Vec<f64>,
        from: Vec<usize>,
        to: Vec<usize>,
        node_out: Vec<Vec<usize>>,
        node_in: Vec<Vec<usize>>,
    ) -> Self {
        let boxes = shapes
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let mut lo = [f64::INFINITY; 2];
                let mut hi = [f64::NEG_INFINITY; 2];
                for p in s {
                    lo = [lo[0].min(p[0]), lo[1].min(p[1])];
                    hi = [hi[0].max(p[0]), hi[1].max(p[1])];
                }
                GeomWithData::new(Rectangle::from_corners(lo, hi), i)
            })
            .collect();
        let length = shapes.iter().map(|s| poly_len(s)).collect();
        Network { shapes, speed, from, to, node_out, node_in, length, tree: RTree::bulk_load(boxes) }
    }

    /// `Net.getNeighboringEdges(x, y, r)`: the edges nearer than `r`, sorted by edge index.
    pub fn neighboring_edges(&self, x: f64, y: f64, r: f64) -> Vec<(usize, f64)> {
        let env = AABB::from_corners([x - r, y - r], [x + r, y + r]);
        let mut v: Vec<(usize, f64)> = self
            .tree
            .locate_in_envelope_intersecting(&env)
            .filter_map(|g| {
                let d = sumo_dist_to_poly([x, y], &self.shapes[g.data]);
                (d < r).then_some((g.data, d))
            })
            .collect();
        v.sort_by_key(|e| e.0);
        v
    }

    /// `network.combineShapesSumo(edge, fromedge, edge_reverse, from_reverse)`.
    fn combine(&self, e: usize, from: Option<usize>, rev: bool, from_rev: bool) -> Vec<P> {
        let s = &self.shapes[e];
        let mut out = Vec::with_capacity(s.len() + 1);
        if let Some(f) = from {
            let fs = &self.shapes[f];
            let p = if !from_rev || !rev { fs[fs.len() - 1] } else { fs[0] };
            let first = if rev { s[s.len() - 1] } else { s[0] };
            if p != first {
                out.push(p);
            }
        }
        if rev {
            out.extend(s.iter().rev());
        } else {
            out.extend(s.iter());
        }
        out
    }
}

// ---------------------------------------------------------------------------------------------
// Road interpolation (interpolation.py, roadInterpolation): the points of a long gap go along the
// shortest road path
// ---------------------------------------------------------------------------------------------

const SNAP_RADIUS: f64 = 50.0;
const SNAP_COUNT: usize = 3;

#[derive(Clone, Copy, Debug)]
pub struct RoadParams {
    pub min_gap: f64,
    pub detour_factor: f64,
    pub detour_m: f64,
}

/// `_cut`: the shape points after the offset lo, up to the offset hi.
fn cut(shape: &[P], lo: f64, hi: f64, out: &mut Vec<P>) {
    let mut acc = 0.0;
    for w in shape.windows(2) {
        let d = dist(w[0], w[1]);
        if acc + d > lo && acc < hi && acc + d <= hi {
            out.push(w[1]);
        }
        acc += d;
    }
}

/// `_along`: the point and the bearing at the distance s along a polyline.
fn along(poly: &[P], s: f64) -> (P, f64) {
    let mut acc = 0.0;
    for w in poly.windows(2) {
        let d = dist(w[0], w[1]);
        if d > 0.0 && acc + d >= s {
            let f = (s - acc) / d;
            return ([w[0][0] + f * (w[1][0] - w[0][0]), w[0][1] + f * (w[1][1] - w[0][1])], bearing(w[0], w[1]));
        }
        acc += d;
    }
    let n = poly.len();
    (poly[n - 1], bearing(poly[n - 2], poly[n - 1]))
}

impl Network {
    /// `RoadPath.snap`: the nearest edges (distance, edge, offset, point on the edge), sorted by (distance, edge).
    fn snap(&self, p: P) -> Vec<(f64, usize, f64, P)> {
        let mut v: Vec<(f64, usize, f64, P)> = self
            .neighboring_edges(p[0], p[1], SNAP_RADIUS)
            .into_iter()
            .map(|(e, _)| {
                let (off, q) = poly_offset(p, &self.shapes[e]);
                (dist(p, q), e, off, q)
            })
            .collect();
        // the two directions of a road are at the same distance; the last bit of the distance can
        // differ between Python and Rust, so the sort uses the distance in micrometres, then the edge index
        v.sort_by_key(|c| ((c.0 * 1e6 + 0.5).floor() as i64, c.1));
        v.truncate(SNAP_COUNT);
        v
    }

    /// `RoadPath.node_path`: (length, edges) from the end of ea to the start of eb, or None.
    /// The heap order is (distance, push number), as in the Python heapq.
    fn node_path(&self, ea: usize, eb: usize, limit: f64) -> Option<(f64, Vec<usize>)> {
        use std::cmp::Reverse;
        use std::collections::{BinaryHeap, HashMap};
        let (src, dst) = (self.to[ea], self.from[eb]);
        if src == dst {
            return Some((0.0, Vec::new()));
        }
        let mut dist_of: HashMap<usize, f64> = HashMap::from([(src, 0.0)]);
        let mut prev: HashMap<usize, usize> = HashMap::new();
        // the distances are 0 or more, so the bits of the f64 sort as the numbers
        let mut heap = BinaryHeap::from([Reverse((0f64.to_bits(), 0u64, src))]);
        let mut n = 1u64;
        while let Some(Reverse((db, _, u))) = heap.pop() {
            let d = f64::from_bits(db);
            if d > *dist_of.get(&u).unwrap_or(&f64::INFINITY) || d > limit {
                continue;
            }
            if u == dst {
                let mut edges = Vec::new();
                let mut w = u;
                while let Some(&e) = prev.get(&w) {
                    edges.push(e);
                    w = self.from[e];
                }
                edges.reverse();
                return Some((d, edges));
            }
            for &e in &self.node_out[u] {
                let (v, nd) = (self.to[e], d + self.length[e]);
                if nd < *dist_of.get(&v).unwrap_or(&f64::INFINITY) {
                    dist_of.insert(v, nd);
                    prev.insert(v, e);
                    n += 1;
                    heap.push(Reverse((nd.to_bits(), n, v)));
                }
            }
        }
        None
    }

    /// `RoadPath.polyline`: the road polyline from near a to near b, or None.
    pub fn road_polyline(&self, a: P, b: P) -> Option<Vec<P>> {
        let air = dist(a, b);
        let limit = (3.0 * air).max(air + 300.0);
        let mut best: Option<(f64, usize, f64, P, usize, f64, P, Option<Vec<usize>>)> = None;
        let sb = self.snap(b);
        for &(da, ea, oa, qa) in &self.snap(a) {
            for &(db, eb, ob, qb) in &sb {
                let (length, mid) = if ea == eb && ob >= oa {
                    (ob - oa, None)
                } else {
                    match self.node_path(ea, eb, limit) {
                        Some((l, m)) => ((self.length[ea] - oa) + l + ob, Some(m)),
                        None => continue,
                    }
                };
                let score = length + 2.0 * (da + db);
                if best.as_ref().map_or(true, |bb| score < bb.0) {
                    best = Some((score, ea, oa, qa, eb, ob, qb, mid));
                }
            }
        }
        let (_, ea, oa, qa, eb, ob, qb, mid) = best?;
        let mut core = vec![qa];
        match mid {
            None => cut(&self.shapes[ea], oa, ob, &mut core),
            Some(m) => {
                cut(&self.shapes[ea], oa, f64::INFINITY, &mut core);
                for e in m {
                    core.extend_from_slice(&self.shapes[e][1..]);
                }
                cut(&self.shapes[eb], 0.0, ob, &mut core);
            }
        }
        core.push(qb);
        core.dedup();
        (core.len() > 1).then_some(core)
    }
}

/// `roadInterpolation`: the Bezier interpolation, but a gap of `min_gap` seconds or more goes
/// along the shortest road path (when it is no detour). The labels are the positions.
#[allow(clippy::too_many_arguments)]
pub fn interpolate_road(
    net: &Network,
    x: &[f64],
    y: &[f64],
    ts: &[f64],
    speed: &[f64],
    brg: &[f64],
    stop: &[i64],
    sr: f64,
    rp: RoadParams,
) -> Interp {
    let n = x.len();
    let mut out = Interp::default();
    for i in 0..n.saturating_sub(1) {
        let (a, b, dt) = ([x[i], y[i]], [x[i + 1], y[i + 1]], ts[i + 1] - ts[i]);
        let mut poly = None;
        if !(stop[i] > 0 && stop[i + 1] > 0) && dt >= rp.min_gap {
            poly = net.road_polyline(a, b);
            if poly.as_ref().is_some_and(|p| poly_len(p) > rp.detour_factor * dist(a, b) + rp.detour_m) {
                poly = None;
            }
        }
        match poly {
            None => {
                let ka = Knot { p: a, ts: ts[i], speed: speed[i], bearing: brg[i], stop: stop[i] };
                let kb = Knot { p: b, ts: ts[i + 1], speed: speed[i + 1], bearing: brg[i + 1], stop: stop[i + 1] };
                let mut seg = Interp::default();
                bezier_segment(ka, kb, sr, &mut seg);
                for k in 0..seg.x.len() {
                    if (seg.ts[k] as f64) < ts[i + 1] {
                        out.push([seg.x[k], seg.y[k]], seg.ts[k], seg.speed[k], seg.bearing[k], seg.origin[k], seg.stop[k]);
                    }
                }
            }
            Some(poly) => {
                let length = poly_len(&poly);
                let mut k = 0.0;
                while k * sr < dt {
                    let (mut q, br) = along(&poly, length * (k * sr) / dt);
                    if k == 0.0 {
                        q = a; // the GPS point stays; it gets the bearing of the road
                    }
                    out.push(q, (ts[i] + k * sr) as i64, length / dt, br, k == 0.0, 0);
                    k += 1.0;
                }
            }
        }
    }
    if n > 0 {
        out.push([x[n - 1], y[n - 1]], ts[n - 1] as i64, speed[n - 1], brg[n - 1], true, stop[n - 1]);
    }
    out.label = (0..out.x.len() as i64).collect();
    out
}

// ---------------------------------------------------------------------------------------------
// MapMatcher.match (mapmatching.py): a beam search
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub max_gps_error: f64,
    pub max_map_error: f64,
    pub map_one_way_fix: bool,
    pub u_turn_on_oneway: bool,
    pub loop_: bool,
    pub diff_gps_error: f64,
    /// The number of hypotheses that the matcher keeps (`BEAM_WIDTH`).
    pub beam_width: usize,
    /// The weights of the cost: bearing error, distance to the edge, air distance error, road
    /// distance error, and the penalty for an edge in the reverse direction.
    pub w_bearing: f64,
    pub w_dist: f64,
    pub w_air: f64,
    pub w_road: f64,
    pub w_reverse: f64,
}

/// One GPS point of the input. `label` is the DataFrame index label.
#[derive(Clone, Copy, Debug)]
pub struct Pt {
    pub label: i64,
    pub x: f64,
    pub y: f64,
    pub speed: f64,
    pub bearing: f64,
    /// The weight factor of the bearing, distance and air distance terms (`point_weights`; 1 = normal).
    pub w: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PathItem {
    pub edge: usize,
    pub rev: bool,
    pub len: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision {
    Stay = 0,
    Change = 1,
    NoDecision = 2,
}

/// One row of `MapMatcher.matchdf`. The columns that are copies of the input row come from `pos`.
#[derive(Clone, Copy, Debug)]
pub struct Matched {
    pub pos: usize,
    pub label: i64,
    pub x: f64,
    pub y: f64,
    pub rd: f64,
    pub dist: f64,
    pub from_edge: Option<usize>,
    pub predict_distance: f64,
    pub matched_road_distance: f64,
    pub cost_air: f64,
    pub decision: Decision,
    pub matchbearing: f64,
    pub edge: usize,
    pub offset: f64,
    pub distbearing: f64,
    pub edge_length: f64,
    pub edge_rev: bool,
    pub from_rev: bool,
    pub cost: f64,
}

/// Insert into a set of (edge, reverse) sorted by edge; an edge already there gets the new flag.
fn set_insert(v: &mut Vec<(usize, bool)>, e: usize, rev: bool) {
    match v.binary_search_by_key(&e, |c| c.0) {
        Ok(i) => v[i].1 = rev,
        Err(i) => v.insert(i, (e, rev)),
    }
}

/// A hypothesis of the beam search. `path` and `matched` point into the node lists of the matcher
/// (linked lists: the hypotheses share their first parts).
#[derive(Clone, Copy, Debug)]
struct Hyp {
    cost: f64,
    edge: Option<usize>,
    rev: bool,
    offset: f64,
    path: Option<usize>,
    matched: Option<usize>,
}

struct PNode {
    item: PathItem,
    prev: Option<usize>,
}

struct MNode {
    m: Matched,
    prev: Option<usize>,
}

/// A child before it goes into the beam: a new path item is made only for the children that stay.
struct Child {
    cost: f64,
    edge: usize,
    rev: bool,
    offset: f64,
    path: Option<usize>,
    new_item: Option<PathItem>,
    m: Matched,
    prev_m: Option<usize>,
}

/// `Done` after `match_trace`; `Running` while `push` adds points.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Running,
    Done,
}

/// The state of `MapMatcher.match`. `match_trace` runs it on a full trace; `push` adds one point
/// (the online use). Both give the same result.
pub struct Matcher {
    net: Arc<Network>,
    p: Params,
    pts: Vec<Pt>,
    beam: Vec<Hyp>,
    pnodes: Vec<PNode>,
    mnodes: Vec<MNode>,
    pub status: Status,
}

impl Matcher {
    pub fn new(net: Arc<Network>, p: Params) -> Self {
        Matcher {
            net,
            p,
            pts: Vec::new(),
            beam: Vec::new(),
            pnodes: Vec::new(),
            mnodes: Vec::new(),
            status: Status::Running,
        }
    }

    fn radius(&self) -> f64 {
        self.p.max_gps_error + self.p.max_map_error
    }

    /// `MapMatcher.match(sample_gps)` on the full trace.
    pub fn match_trace(&mut self, pts: Vec<Pt>) -> Result<Status, String> {
        *self = Matcher::new(Arc::clone(&self.net), self.p);
        for pt in pts {
            self.pts.push(pt);
            self.step(self.pts.len() - 1)?;
        }
        self.status = Status::Done;
        Ok(self.status)
    }

    /// Add one point and match it. The label of the point is its position.
    pub fn push(&mut self, x: f64, y: f64, speed: f64, bearing: f64, w: f64) -> Result<Status, String> {
        let label = self.pts.len() as i64;
        self.pts.push(Pt { label, x, y, speed, bearing, w });
        self.step(self.pts.len() - 1)?;
        Ok(self.status)
    }

    /// The matched points of the best hypothesis, first to last.
    pub fn matched(&self) -> Vec<Matched> {
        let mut v = Vec::new();
        let mut n = self.beam.first().and_then(|h| h.matched);
        while let Some(i) = n {
            v.push(self.mnodes[i].m);
            n = self.mnodes[i].prev;
        }
        v.reverse();
        v
    }

    /// The path of the best hypothesis, first to last.
    pub fn path(&self) -> Vec<PathItem> {
        self.path_of(self.beam.first().and_then(|h| h.path))
    }

    /// The number of matched points of the best hypothesis.
    pub fn matched_len(&self) -> usize {
        let mut k = 0;
        let mut n = self.beam.first().and_then(|h| h.matched);
        while let Some(i) = n {
            k += 1;
            n = self.mnodes[i].prev;
        }
        k
    }

    /// The edge of the last matched point of the best hypothesis.
    pub fn last_edge(&self) -> Option<usize> {
        self.beam.first().and_then(|h| h.edge)
    }

    fn path_of(&self, mut n: Option<usize>) -> Vec<PathItem> {
        let mut v = Vec::new();
        while let Some(i) = n {
            v.push(self.pnodes[i].item);
            n = self.pnodes[i].prev;
        }
        v.reverse();
        v
    }

    /// `MapMatcher.outgoinglist(edge, reverse, path)`.
    fn outgoing(&self, e: usize, rev: bool, path: Option<usize>) -> Vec<(usize, bool)> {
        let net = &*self.net;
        let mut v: Vec<(usize, bool)> = Vec::new();
        let node = if self.p.map_one_way_fix && rev { net.from[e] } else { net.to[e] };
        let mut out: Vec<usize> = net.node_out[node].clone();
        if self.p.map_one_way_fix {
            out.extend(net.node_in[node].iter().copied());
            if !self.p.u_turn_on_oneway {
                if let Some(i) = out.iter().position(|&c| c == e) {
                    out.remove(i);
                }
            }
        }
        if !self.p.loop_ {
            for it in self.path_of(path) {
                if let Some(i) = out.iter().position(|&c| c == it.edge) {
                    out.remove(i);
                }
            }
        }
        for c in out {
            let r = self.p.map_one_way_fix && !net.node_out[node].contains(&c);
            set_insert(&mut v, c, r);
        }
        v
    }

    /// `MapMatcher.step`: one point of the beam search.
    fn step(&mut self, pos: usize) -> Result<(), String> {
        let net_arc = Arc::clone(&self.net);
        let net = &*net_arc;
        let pt = self.pts[pos];
        let mut children: Vec<Child> = Vec::new();
        for h in self.beam.clone() {
            let le = h.edge.expect("a hypothesis in the beam has an edge");
            let last = &self.pnodes[h.path.expect("a hypothesis in the beam has a path")].item;
            let remind = last.len - h.offset;
            let d = if remind < (pt.speed * 1.0) - self.p.diff_gps_error {
                Decision::Change
            } else if remind >= (net.speed[le] * 1.0) + self.p.diff_gps_error {
                Decision::Stay
            } else {
                Decision::NoDecision
            };
            let edges = match d {
                Decision::Stay => vec![(le, h.rev)],
                Decision::Change => self.outgoing(le, h.rev, h.path),
                Decision::NoDecision => {
                    let mut e = self.outgoing(le, h.rev, h.path);
                    set_insert(&mut e, le, h.rev);
                    e
                }
            };
            self.extend(&h, pos, d, &edges, &mut children)?;
        }
        if children.is_empty() {
            // no hypothesis can continue: start again from the edges near the point (a gap)
            let base = self.beam.first().copied();
            let start = Hyp {
                cost: base.map_or(0.0, |b| b.cost),
                edge: None,
                rev: false,
                offset: 0.0,
                path: base.and_then(|b| b.path),
                matched: base.and_then(|b| b.matched),
            };
            let edges: Vec<(usize, bool)> =
                net.neighboring_edges(pt.x, pt.y, self.radius()).into_iter().map(|(e, _)| (e, false)).collect();
            self.extend(&start, pos, Decision::Change, &edges, &mut children)?;
            if children.is_empty() {
                return Ok(()); // no edge near the point: skip it
            }
        }
        children.sort_by(|a, b| a.cost.partial_cmp(&b.cost).unwrap_or(std::cmp::Ordering::Equal));
        let mut beam: Vec<Hyp> = Vec::with_capacity(self.p.beam_width);
        for c in children {
            if beam.iter().any(|b| b.edge == Some(c.edge) && b.rev == c.rev) {
                continue;
            }
            let path = match c.new_item {
                Some(item) => {
                    self.pnodes.push(PNode { item, prev: c.path });
                    Some(self.pnodes.len() - 1)
                }
                None => c.path,
            };
            self.mnodes.push(MNode { m: c.m, prev: c.prev_m });
            beam.push(Hyp {
                cost: c.cost,
                edge: Some(c.edge),
                rev: c.rev,
                offset: c.offset,
                path,
                matched: Some(self.mnodes.len() - 1),
            });
            if beam.len() == self.p.beam_width {
                break;
            }
        }
        self.beam = beam;
        Ok(())
    }

    /// `MapMatcher._extend`: the children of `h` on the candidate edges (sorted by edge index).
    fn extend(&self, h: &Hyp, pos: usize, decision: Decision, edges: &[(usize, bool)], out: &mut Vec<Child>) -> Result<(), String> {
        let net = &*self.net;
        let pt = self.pts[pos];
        let xy = [pt.x, pt.y];
        let last_path = h.path.map(|i| self.pnodes[i].item);
        let prev_path = h.path.and_then(|i| self.pnodes[i].prev).map(|i| self.pnodes[i].item);
        let last_m = h.matched.map(|i| self.mnodes[i].m);
        for &(e, rev) in edges {
            let (temp, temp_rev) = if h.edge.is_some() && Some(e) != h.edge {
                (h.edge, h.rev)
            } else if let (Some(_), Some(pp)) = (h.edge, prev_path) {
                (Some(pp.edge), pp.rev) // stay: the edge before it
            } else {
                (None, false)
            };
            let shape = net.combine(e, temp, rev, temp_rev);
            let length = poly_len(&shape);
            let (offset, mp) = poly_offset(xy, &shape);
            let d = dist(mp, xy);
            if d > self.radius() {
                continue;
            }
            let mb = offset_bearing(&shape, offset)?;
            let ad = (pt.bearing - mb).abs();
            let db = if 360.0 - ad < ad { 360.0 - ad } else { ad };
            let (lastedge, lastoffset, last_sample, last_matched, last_len) = match last_m {
                Some(m) => {
                    let q = self.pts[m.pos];
                    (m.edge, m.offset, [q.x, q.y], [m.x, m.y], m.edge_length)
                }
                None => (e, offset, xy, mp, 0.0),
            };
            let cost_air = (dist(last_sample, xy) - dist(last_matched, mp)).abs();
            let predict = pt.speed * 1.0;
            let mrd = if e == lastedge {
                let w = offset - lastoffset;
                if w > 0.0 { w } else { 0.0 }
            } else {
                (last_len - lastoffset) + offset
            };
            let rd = (mrd - predict).abs();
            let p = &self.p;
            let mut cost = if rev { p.w_reverse } else { 0.0 };
            cost += p.w_bearing * db * pt.w + p.w_dist * d * pt.w + p.w_air * cost_air * pt.w + p.w_road * rd;
            let new_item = if Some(e) == h.edge {
                None
            } else {
                let len = poly_len(&net.combine(e, last_path.map(|p| p.edge), rev, last_path.is_some_and(|p| p.rev)));
                Some(PathItem { edge: e, rev, len })
            };
            out.push(Child {
                cost: h.cost + cost,
                edge: e,
                rev,
                offset,
                path: h.path,
                new_item,
                m: Matched {
                    pos,
                    label: pt.label,
                    x: mp[0],
                    y: mp[1],
                    rd,
                    dist: d,
                    from_edge: temp,
                    predict_distance: predict,
                    matched_road_distance: mrd,
                    cost_air,
                    decision,
                    matchbearing: mb,
                    edge: e,
                    offset,
                    distbearing: db,
                    edge_length: length,
                    edge_rev: rev,
                    from_rev: temp_rev,
                    cost,
                },
                prev_m: h.matched,
            });
        }
        Ok(())
    }
}
