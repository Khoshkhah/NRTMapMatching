//! Python module `nrt_matcher`: the Rust engine of NRTMapMatching.
//!
//! `sources/cleandata.py`, `sources/interpolation.py` and `sources/mapmatching.py` call it when it
//! is installed (see `sources/engine.py`). The arguments are plain lists; the network comes as
//! arrays in the sequence of `Net._edgeidlist`.

pub mod core;

use crate::core as c;
use pyo3::exceptions::{PyIndexError, PyKeyError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyDict;
use std::sync::Arc;

type RowsOut = (Vec<f64>, Vec<f64>, Vec<i64>, Vec<f64>, Vec<f64>, Vec<i64>);

fn rows_out(r: c::Rows) -> RowsOut {
    (r.x, r.y, r.ts, r.speed, r.bearing, r.stop)
}

/// The Python code raises IndexError and KeyError where it pops an empty list or removes a missing key.
fn err(e: String) -> PyErr {
    if e == "pop from empty list" || e == "list index out of range" {
        PyIndexError::new_err(e)
    } else if e == "KeyError" {
        PyKeyError::new_err(e)
    } else {
        PyValueError::new_err(e)
    }
}

/// `cleaningData` after the sort, the group by timestamp and the projection.
/// Returns the columns (x, y, timestamp, speed, bearing, stopindex).
#[pyfunction]
fn clean(
    x: Vec<f64>,
    y: Vec<f64>,
    ts: Vec<i64>,
    speed: Vec<f64>,
    bearing: Vec<f64>,
    min_speed_for_bearing: f64,
    max_speed_for_outlier: f64,
) -> PyResult<RowsOut> {
    if x.len() < 2 {
        return Err(err("observations is too small!".into()));
    }
    Ok(rows_out(c::clean(&x, &y, &ts, &speed, &bearing, min_speed_for_bearing, max_speed_for_outlier)))
}

/// `richdata` after the projection. Returns the columns (x, y, timestamp, speed, bearing, stopindex).
#[pyfunction]
#[pyo3(signature = (x, y, ts, alpha, min_speed, max_speed_for_outlier, min_speed_for_bearing, smooth_max_gap=None))]
#[allow(clippy::too_many_arguments)]
fn enrich(
    x: Vec<f64>,
    y: Vec<f64>,
    ts: Vec<i64>,
    alpha: f64,
    min_speed: f64,
    max_speed_for_outlier: f64,
    min_speed_for_bearing: f64,
    smooth_max_gap: Option<f64>,
) -> PyResult<RowsOut> {
    if x.is_empty() {
        return Err(err("no observations".into()));
    }
    c::enrich(&x, &y, &ts, alpha, min_speed, max_speed_for_outlier, min_speed_for_bearing, smooth_max_gap)
        .map(rows_out)
        .map_err(err)
}

/// `interpolateTrajectory`. Returns (index labels, x, y, timestamp, speed, bearing, origin, stopindex).
#[pyfunction]
#[allow(clippy::type_complexity)]
fn interpolate(
    x: Vec<f64>,
    y: Vec<f64>,
    ts: Vec<f64>,
    speed: Vec<f64>,
    bearing: Vec<f64>,
    stop: Vec<i64>,
    sample_rate: f64,
) -> PyResult<(Vec<i64>, Vec<f64>, Vec<f64>, Vec<i64>, Vec<f64>, Vec<f64>, Vec<bool>, Vec<i64>)> {
    if x.len() < 3 {
        return Err(err("interpolateTrajectory needs 3 rows or more".into()));
    }
    let r = c::interpolate(&x, &y, &ts, &speed, &bearing, &stop, sample_rate);
    Ok((r.label, r.x, r.y, r.ts, r.speed, r.bearing, r.origin, r.stop))
}

/// The road network: edge shapes, edge speeds, the from and to node of each edge, and the
/// outgoing and incoming edges of each node (edge and node indexes).
#[pyclass(frozen)]
struct Network(Arc<c::Network>);

#[pymethods]
impl Network {
    #[new]
    fn new(
        shapes: Vec<Vec<(f64, f64)>>,
        speed: Vec<f64>,
        from_node: Vec<usize>,
        to_node: Vec<usize>,
        node_out: Vec<Vec<usize>>,
        node_in: Vec<Vec<usize>>,
    ) -> PyResult<Self> {
        let n = shapes.len();
        if speed.len() != n || from_node.len() != n || to_node.len() != n || node_out.len() != node_in.len() {
            return Err(err("the network arrays do not agree".into()));
        }
        if shapes.iter().any(|s| s.len() < 2) {
            return Err(err("each edge shape needs 2 points or more".into()));
        }
        let shapes = shapes.into_iter().map(|s| s.into_iter().map(|(x, y)| [x, y]).collect()).collect();
        Ok(Network(Arc::new(c::Network::new(shapes, speed, from_node, to_node, node_out, node_in))))
    }

    /// `RoadPath.polyline((ax, ay), (bx, by))`: the road polyline between two points, or None.
    fn road_polyline(&self, ax: f64, ay: f64, bx: f64, by: f64) -> Option<Vec<(f64, f64)>> {
        self.0.road_polyline([ax, ay], [bx, by]).map(|v| v.into_iter().map(|p| (p[0], p[1])).collect())
    }

    /// `Net.getNeighboringEdges(x, y, r)`: [(edge index, distance)], sorted by edge index.
    fn neighboring_edges(&self, x: f64, y: f64, r: f64) -> Vec<(usize, f64)> {
        self.0.neighboring_edges(x, y, r)
    }
}

/// `MapMatcher.match` (beam search): `match_trace` for a full trace, `push` for one new point (online use).
#[pyclass]
struct Matcher(c::Matcher);

#[pymethods]
impl Matcher {
    #[new]
    #[pyo3(signature = (network, max_gps_error, max_map_error, map_one_way_fix, u_turn_on_oneway, loop_,
                        diff_gps_error, beam_width=20, w_bearing=1.0, w_dist=30.0, w_air=10.0, w_road=5.0,
                        w_reverse=100000.0))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        network: &Network,
        max_gps_error: f64,
        max_map_error: f64,
        map_one_way_fix: bool,
        u_turn_on_oneway: bool,
        loop_: bool,
        diff_gps_error: f64,
        beam_width: usize,
        w_bearing: f64,
        w_dist: f64,
        w_air: f64,
        w_road: f64,
        w_reverse: f64,
    ) -> PyResult<Self> {
        if beam_width == 0 {
            return Err(err("beam_width must be 1 or more".into()));
        }
        let p = c::Params {
            max_gps_error,
            max_map_error,
            map_one_way_fix,
            u_turn_on_oneway,
            loop_,
            diff_gps_error,
            beam_width,
            w_bearing,
            w_dist,
            w_air,
            w_road,
            w_reverse,
        };
        Ok(Matcher(c::Matcher::new(Arc::clone(&network.0), p)))
    }

    /// Match a full trace. `labels` is the DataFrame index; `weights` the weight factor of each
    /// point (`MapMatcher.point_weights`; None = 1 for all). Returns 1.
    #[pyo3(signature = (labels, x, y, speed, bearing, weights=None))]
    #[allow(clippy::too_many_arguments)]
    fn match_trace(
        &mut self,
        py: Python<'_>,
        labels: Vec<i64>,
        x: Vec<f64>,
        y: Vec<f64>,
        speed: Vec<f64>,
        bearing: Vec<f64>,
        weights: Option<Vec<f64>>,
    ) -> PyResult<i32> {
        let n = x.len();
        let w = weights.unwrap_or_else(|| vec![1.0; n]);
        if labels.len() != n || y.len() != n || speed.len() != n || bearing.len() != n || w.len() != n {
            return Err(err("the point arrays do not agree".into()));
        }
        let pts = (0..n)
            .map(|i| c::Pt { label: labels[i], x: x[i], y: y[i], speed: speed[i], bearing: bearing[i], w: w[i] })
            .collect();
        let m = &mut self.0;
        py.detach(|| m.match_trace(pts)).map_err(err)?;
        Ok(1)
    }

    /// Add one point and match it (`weight`: the weight factor of the point). Returns the number of
    /// matched points of the best hypothesis.
    #[pyo3(signature = (x, y, speed, bearing, weight=1.0))]
    fn push(&mut self, x: f64, y: f64, speed: f64, bearing: f64, weight: f64) -> PyResult<usize> {
        self.0.push(x, y, speed, bearing, weight).map_err(err)?;
        Ok(self.0.matched_len())
    }

    /// The edge index of the last matched point of the best hypothesis, or None.
    fn last_edge(&self) -> Option<usize> {
        self.0.last_edge()
    }

    /// The path of the best hypothesis as [(edge index, reverse, length)].
    fn path(&self) -> Vec<(usize, bool, f64)> {
        self.0.path().iter().map(|p| (p.edge, p.rev, p.len)).collect()
    }

    /// The matched points of the best hypothesis as columns. `pos` is the input row; `from_edge`
    /// is -1 for none; `decision` is 0 STAY, 1 CHANGE, 2 NODECISION.
    fn result<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let m = self.0.matched();
        let d = PyDict::new(py);
        macro_rules! col {
            ($name:literal, $f:expr) => {
                d.set_item($name, m.iter().map($f).collect::<Vec<_>>())?;
            };
        }
        col!("pos", |r| r.pos);
        col!("label", |r| r.label);
        col!("x", |r| r.x);
        col!("y", |r| r.y);
        col!("rd", |r| r.rd);
        col!("dist", |r| r.dist);
        col!("from_edge", |r| r.from_edge.map_or(-1, |e| e as i64));
        col!("predict_distance", |r| r.predict_distance);
        col!("matched_road_distance", |r| r.matched_road_distance);
        col!("cost_air", |r| r.cost_air);
        col!("decision", |r| r.decision as u8);
        col!("matchbearing", |r| r.matchbearing);
        col!("edge", |r| r.edge);
        col!("offset", |r| r.offset);
        col!("distbearing", |r| r.distbearing);
        col!("edge_length", |r| r.edge_length);
        col!("edge_reverse", |r| r.edge_rev);
        col!("from_edge_reverse", |r| r.from_rev);
        col!("cost", |r| r.cost);
        Ok(d)
    }
}

/// `interpolateTrajectory(traj, net=net)` (road interpolation). Returns (index labels, x, y,
/// timestamp, speed, bearing, origin, stopindex).
#[pyfunction]
#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn interpolate_road(
    network: &Network,
    x: Vec<f64>,
    y: Vec<f64>,
    ts: Vec<f64>,
    speed: Vec<f64>,
    bearing: Vec<f64>,
    stop: Vec<i64>,
    sample_rate: f64,
    min_gap: f64,
    detour_factor: f64,
    detour_m: f64,
) -> PyResult<(Vec<i64>, Vec<f64>, Vec<f64>, Vec<i64>, Vec<f64>, Vec<f64>, Vec<bool>, Vec<i64>)> {
    let n = x.len();
    if y.len() != n || ts.len() != n || speed.len() != n || bearing.len() != n || stop.len() != n {
        return Err(err("the point arrays do not agree".into()));
    }
    if !(sample_rate > 0.0) {
        return Err(err("sample_rate must be more than 0".into()));
    }
    let rp = c::RoadParams { min_gap, detour_factor, detour_m };
    let r = c::interpolate_road(&network.0, &x, &y, &ts, &speed, &bearing, &stop, sample_rate, rp);
    Ok((r.label, r.x, r.y, r.ts, r.speed, r.bearing, r.origin, r.stop))
}

#[pymodule]
fn nrt_matcher(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(clean, m)?)?;
    m.add_function(wrap_pyfunction!(enrich, m)?)?;
    m.add_function(wrap_pyfunction!(interpolate, m)?)?;
    m.add_function(wrap_pyfunction!(interpolate_road, m)?)?;
    m.add_class::<Network>()?;
    m.add_class::<Matcher>()?;
    Ok(())
}
