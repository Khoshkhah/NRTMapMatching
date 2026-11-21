//! The map matching of NRTMapMatching in plain Rust.
//!
//! Each function is a line-by-line port of the Python code in `sources/`. The float operations
//! are in the same sequence as in Python, so that the two engines give the same numbers. Where
//! the Python code has a quirk (for example it mixes DataFrame labels and positions), the port
//! keeps the quirk: the results must stay equal.

use rstar::primitives::{GeomWithData, Rectangle};
use rstar::{RTree, AABB};
use std::sync::Arc;
use std::time::{Duration, Instant};

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
        Network { shapes, speed, from, to, node_out, node_in, tree: RTree::bulk_load(boxes) }
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
// MapMatcher.match (mapmatching.py)
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub max_gps_error: f64,
    pub max_map_error: f64,
    pub map_one_way_fix: bool,
    pub u_turn_on_oneway: bool,
    pub loop_: bool,
    pub diff_gps_error: f64,
    /// `MAX_RUNNING_TIME` in seconds. `match_trace` stops at this limit and keeps the path found so
    /// far; `push` stops its step back at this limit and continues it at the next push.
    pub max_running_time: f64,
}

/// One GPS point of the input. `label` is the DataFrame index label (Python mixes it with the position).
#[derive(Clone, Copy, Debug)]
pub struct Pt {
    pub label: i64,
    pub x: f64,
    pub y: f64,
    pub speed: f64,
    pub bearing: f64,
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

/// A decision point: the edges not yet tried, each with its "reverse" flag (sorted by edge).
struct Branch {
    index: i64,
    result: Vec<(usize, bool)>,
    last_edge: Option<usize>,
}

fn set_insert(v: &mut Vec<(usize, bool)>, e: usize, rev: bool) {
    match v.binary_search_by_key(&e, |c| c.0) {
        Ok(i) => v[i].1 = rev,
        Err(i) => v.insert(i, (e, rev)),
    }
}

fn set_remove(v: &mut Vec<(usize, bool)>, e: usize) -> bool {
    match v.binary_search_by_key(&e, |c| c.0) {
        Ok(i) => {
            v.remove(i);
            true
        }
        Err(_) => false,
    }
}

/// `Done` after `match_trace`; `Running` while `push` adds points.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Running,
    Done,
}

enum Step {
    Accept(i64),
    Back(i64),
}

/// The state of `MapMatcher.match`. `match_trace` runs it on a full trace; `push` adds one point
/// and continues it (the online use).
pub struct Matcher {
    net: Arc<Network>,
    p: Params,
    pub path: Vec<PathItem>,
    pts: Vec<Pt>,
    branches: Vec<Branch>,
    pub matched: Vec<Matched>,
    myindex: i64,
    last_edge: Option<usize>,
    last_offset: f64,
    last_rev: bool,
    pub status: Status,
    /// True when the last `match_trace` stopped at `max_running_time` (the path is only a part).
    pub timed_out: bool,
}

impl Matcher {
    pub fn new(net: Arc<Network>, p: Params, path: Vec<PathItem>) -> Self {
        Matcher {
            net,
            p,
            path,
            pts: Vec::new(),
            branches: Vec::new(),
            matched: Vec::new(),
            myindex: 0,
            last_edge: None,
            last_offset: 0.0,
            last_rev: false,
            status: Status::Running,
            timed_out: false,
        }
    }

    fn radius(&self) -> f64 {
        self.p.max_gps_error + self.p.max_map_error
    }

    /// `MapMatcher.match(sample_gps)` on the full trace.
    pub fn match_trace(&mut self, pts: Vec<Pt>) -> Result<Status, String> {
        self.pts = pts;
        self.start();
        self.run(true)?;
        Ok(self.status)
    }

    /// Add one point and match it (and again the points after a step back). The labels are the positions.
    /// The result equals `match_trace` on the same points, except when a step back occurs at the
    /// last point of the trace: there the Python code stops, and `push` continues.
    pub fn push(&mut self, x: f64, y: f64, speed: f64, bearing: f64) -> Result<Status, String> {
        let label = self.pts.len() as i64;
        self.pts.push(Pt { label, x, y, speed, bearing });
        if self.pts.len() == 1 {
            self.start();
        }
        if self.status == Status::Running {
            self.run(false)?;
        }
        Ok(self.status)
    }

    fn start(&mut self) {
        self.branches = vec![self.start_branch(0, 0)];
        self.myindex = 0;
        self.last_edge = None;
        self.status = Status::Running;
        self.timed_out = false;
    }

    /// `first_point_matching`: a decision point at the point `pos` with the edges near it.
    fn start_branch(&self, pos: usize, index: i64) -> Branch {
        let p = self.pts[pos];
        let result = self.net.neighboring_edges(p.x, p.y, self.radius()).into_iter().map(|(e, _)| (e, false)).collect();
        Branch { index, result, last_edge: None }
    }

    fn run(&mut self, batch: bool) -> Result<(), String> {
        let n = self.pts.len() as i64;
        let t0 = Instant::now();
        while self.myindex < n {
            let mut label = -1;
            let mut broke = false;
            for pos in self.myindex as usize..n as usize {
                label = self.pts[pos].label;
                match self.step(pos)? {
                    Step::Accept(l) => self.myindex = l,
                    Step::Back(i) => {
                        self.myindex = i;
                        broke = true;
                        break;
                    }
                }
            }
            if batch {
                if label == n - 1 {
                    self.myindex = label + 1;
                }
            } else if !broke {
                self.myindex = n;
            }
            if t0.elapsed() > Duration::from_secs_f64(self.p.max_running_time) {
                if batch {
                    self.timed_out = true;
                    self.status = Status::Done;
                }
                return Ok(());
            }
        }
        if batch {
            self.status = Status::Done;
        }
        Ok(())
    }

    /// `MapMatcher.outgoinglist(edge, reverse)`.
    fn outgoing(&self, e: usize, rev: bool) -> Vec<(usize, bool)> {
        let net = &*self.net;
        let mut v: Vec<(usize, bool)> = Vec::new();
        if self.p.map_one_way_fix {
            let node = if rev { net.from[e] } else { net.to[e] };
            let mut out: Vec<usize> = net.node_out[node].iter().chain(net.node_in[node].iter()).copied().collect();
            if !self.p.u_turn_on_oneway {
                if let Some(i) = out.iter().position(|&c| c == e) {
                    out.remove(i);
                }
            }
            if !self.p.loop_ {
                for it in &self.path {
                    if let Some(i) = out.iter().position(|&c| c == it.edge) {
                        out.remove(i);
                    }
                }
            }
            for c in out {
                set_insert(&mut v, c, !net.node_out[node].contains(&c));
            }
        } else {
            // the Python code has no LOOP=False here (NameError); the binding does not come here then
            for &c in &net.node_out[net.to[e]] {
                set_insert(&mut v, c, false);
            }
        }
        v
    }

    fn step(&mut self, pos: usize) -> Result<Step, String> {
        let net_arc = Arc::clone(&self.net);
        let net = &*net_arc;
        let pt = self.pts[pos];
        if self.last_edge != self.branches.last().unwrap().last_edge {
            return Err("error in the algorithm".into());
        }
        let (mut edges, decision) = match self.last_edge {
            Some(le) => {
                let remind = self.path.last().ok_or("list index out of range")?.len - self.last_offset;
                let maxspeed = net.speed[le];
                let d = if remind < (pt.speed * 1.0) - self.p.diff_gps_error {
                    Decision::Change
                } else if remind >= (maxspeed * 1.0) + self.p.diff_gps_error {
                    Decision::Stay
                } else {
                    Decision::NoDecision
                };
                let e = match d {
                    Decision::Stay => vec![(le, self.last_rev)],
                    Decision::Change => self.outgoing(le, self.last_rev),
                    Decision::NoDecision => {
                        let mut e = self.outgoing(le, self.last_rev);
                        set_insert(&mut e, le, self.last_rev);
                        e
                    }
                };
                (e, d)
            }
            None => (self.branches.last().unwrap().result.clone(), Decision::Change),
        };
        if edges.is_empty() {
            // no candidate edge (a dead end, or no edge near the point): the match starts again at
            // this point. The path found before stays. Without an edge near the point, skip it.
            let b = self.start_branch(pos, pt.label);
            edges = b.result.clone();
            self.branches = vec![b];
            self.last_edge = None;
            self.last_rev = false;
            if edges.is_empty() {
                return Ok(Step::Accept(pt.label));
            }
        }

        let xy = [pt.x, pt.y];
        let mut best: Option<Matched> = None;
        for &(e, rev) in &edges {
            let (temp, temp_rev) = if Some(e) != self.last_edge && self.last_edge.is_some() {
                (self.last_edge, self.last_rev)
            } else if self.path.len() < 2 {
                (None, false)
            } else {
                let it = self.path[self.path.len() - 2];
                (Some(it.edge), it.rev)
            };
            let shape = net.combine(e, temp, rev, temp_rev);
            let length = poly_len(&shape);
            let (offset, mp) = poly_offset(xy, &shape);
            let d = dist(mp, xy);
            let mb = offset_bearing(&shape, offset)?;
            let ad = (pt.bearing - mb).abs();
            let db = if 360.0 - ad < ad { 360.0 - ad } else { ad };
            let (lastedge, lastoffset, last_sample, last_matched, last_len) = match self.matched.last() {
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
            let mut cost = if rev { 100000.0 } else { 0.0 };
            cost += 1.0 * db + 30.0 * d + 10.0 * cost_air + 5.0 * rd;
            if best.map_or(true, |b| cost < b.cost) {
                best = Some(Matched {
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
                });
            }
        }
        let b = best.unwrap();

        if b.dist > self.radius() {
            // back to the last decision point that has an edge not yet tried
            set_remove(&mut self.branches.last_mut().unwrap().result, b.edge);
            let mut counter = 0;
            while self.branches.last().is_some_and(|br| br.result.is_empty()) {
                counter += 1;
                self.branches.pop();
                self.path.pop();
            }
            if counter == 0 {
                self.path.pop();
            }
            if self.branches.is_empty() {
                // no decision point is left: the match starts again at this point
                let b = self.start_branch(pos, pt.label);
                self.branches.push(b);
            }
            let top = self.branches.last_mut().unwrap();
            top.last_edge = None;
            let back = top.index;
            while self.matched.last().is_some_and(|m| m.label >= back) {
                self.matched.pop();
            }
            self.last_edge = None;
            self.last_rev = false;
            return Ok(Step::Back(back));
        }

        self.last_offset = b.offset;
        self.matched.push(b);
        let from = self.path.last().map_or((None, false), |it| (Some(it.edge), it.rev));
        if self.last_edge.is_none() {
            let top = self.branches.last_mut().unwrap();
            top.last_edge = Some(b.edge);
            if !set_remove(&mut top.result, b.edge) {
                return Err("KeyError".into());
            }
            let len = poly_len(&net.combine(b.edge, from.0, b.edge_rev, from.1));
            self.path.push(PathItem { edge: b.edge, rev: b.edge_rev, len });
            self.last_edge = Some(b.edge);
            self.last_rev = b.edge_rev;
        } else if Some(b.edge) == self.last_edge {
            // stay on the edge
        } else {
            set_remove(&mut edges, self.last_edge.unwrap());
            set_remove(&mut edges, b.edge);
            let len = poly_len(&net.combine(b.edge, from.0, b.edge_rev, from.1));
            self.path.push(PathItem { edge: b.edge, rev: b.edge_rev, len });
            self.last_edge = Some(b.edge);
            self.last_rev = b.edge_rev;
            self.branches.push(Branch { index: pt.label, result: edges, last_edge: Some(b.edge) });
        }
        if self.last_edge != self.branches.last().unwrap().last_edge {
            return Err("error 2 in the algorithm.".into());
        }
        Ok(Step::Accept(pt.label))
    }
}
