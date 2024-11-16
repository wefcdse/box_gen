use core::f64;
use std::{fs, io::BufWriter};

use box_gen::{
    cacl::{
        lerp::Lerp,
        point::{Point2Trait, PointTrait},
    },
    support_type::Area,
    utils::write_line_to_obj,
};
use rayon::iter::{IntoParallelIterator, ParallelIterator};

fn main() {
    let area = Area::gen_from_obj_file("input.obj", 100, 10., 10., 0.1);
    area.write_to_obj(&mut BufWriter::new(
        fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open("temp/rrt_area.obj")
            .unwrap(),
    ))
    .unwrap();
    area.write_info(
        &mut fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open("temp/info.txt")
            .unwrap(),
    )
    .unwrap();
    let path = {
        let (start, end) = loop {
            let s = area.random_point();
            let e = area.random_point();
            if !area.collide_point(s) && !area.collide_point(e) && area.collide_line(s, e) {
                break (s, e);
            }
        };
        dbg!(start, end);

        // let path = rrt_move::<3, XYZ>(&(), &area, start, end);
        // let mut best_path = path;
        // for _ in 0..10 {
        //     dbg!();
        //     let path = rrt_move::<3, XYZ>(&(), &area, start, end);
        //     if path.len() < best_path.len() {
        //         best_path = path;
        //     }
        // }

        let paths = (0..64)
            .into_par_iter()
            .map(|i| {
                let p = rrt_move::<3, XYZ>(&(), &area, start, end);
                dbg!(i);
                p
            })
            .collect::<Vec<_>>();
        // best_path
        paths
            .into_iter()
            .min_by(|a, b| a.len().cmp(&b.len()))
            .unwrap()
    };
    write_line_to_obj(
        &mut BufWriter::new(
            fs::OpenOptions::new()
                .create(true)
                .write(true)
                .truncate(true)
                .open("temp/rrt_path.obj")
                .unwrap(),
        ),
        &path,
    )
    .unwrap();
}

fn rrt(area: &Area, start: [f64; 3], end: [f64; 3]) -> Vec<[f64; 3]> {
    let step_length = area.block_width() * 4.;
    let mut route = Vec::from([(0, start)]);
    loop {
        let nearest_idx = |p: [f64; 3]| {
            let mut nearest_idx = 0;
            let mut l2 = f64::MAX;
            for (idx, (_, p1)) in route.iter().copied().enumerate() {
                let direction_fix = end.sub(p1).normal().dot(p.sub(p1).normal());
                // dbg!(direction_fix);
                let l = p1.sub(p).length2() * (1.03 - direction_fix);
                if l < l2 {
                    l2 = l;
                    nearest_idx = idx;
                }
            }
            nearest_idx
        };

        let p = area.random_point();
        if area.collide_point(p) {
            continue;
        }
        let (np, ni) = {
            let ni = nearest_idx(p);
            (route[ni].1, ni)
        };
        let next_p = {
            let l = np.sub(p).length();
            let u = (step_length / l).min(1.);
            assert!(u >= 0.);
            (np, p).lerp(u)
        };
        if area.collide_point(next_p) || area.collide_line(np, next_p) {
            continue;
        }
        route.push((ni, next_p));
        if (next_p, end).length2() <= step_length * step_length && !area.collide_line(next_p, end) {
            break;
        }
    }
    let mut path = Vec::from([end]);
    let mut now_idx = route.len() - 1;
    loop {
        let (parent, point) = route[now_idx];
        path.push(point);
        if parent == now_idx {
            break;
        }
        now_idx = parent;
    }
    // path.push(start);
    path.reverse();
    path
}

fn rrt_move<const L: usize, M: AsMove<L>>(
    config: &M::Config,
    area: &Area,
    start: [f64; 3],
    end: [f64; 3],
) -> Vec<[f64; 3]> {
    let moveset = M::new(config);

    let step_length = area.block_width() * 2.5;
    let mut route = Vec::from([(0, start)]);
    let mut counter = 0;
    loop {
        let nearest_idx = |p: [f64; 3]| {
            let mut nearest_idx = 0;
            let mut l2 = f64::MAX;
            for (idx, (_, p1)) in route.iter().copied().enumerate() {
                let direction_fix = end.sub(p1).normal().dot(p.sub(p1).normal());
                // dbg!(direction_fix);
                let l = p1.sub(p).length2() * (1.03 - direction_fix);
                if l < l2 {
                    l2 = l;
                    nearest_idx = idx;
                }
            }
            nearest_idx
        };

        let p = area.random_point();
        // if area.collide_point(p) {
        //     continue;
        // }
        let (base_point, base_idx) = {
            let ni = nearest_idx(p);
            (route[ni].1, ni)
        };

        let direction = p.sub(base_point).normal();
        let moves = moveset.normals(base_point);
        // 选择最接近的方向
        let nearest_idx = moves
            .into_iter()
            .enumerate()
            .fold((f64::MIN, 0), |(max_abs, max_idx), (idx, normal)| {
                let abs = normal.dot(direction).abs();
                if abs > max_abs {
                    (abs, idx)
                } else {
                    (max_abs, max_idx)
                }
            })
            .1;
        // dbg!(nearest_idx);
        let step = moves[nearest_idx].dot(direction).signum() * step_length;
        // dbg!(step);
        let next_p = moveset.apply(base_point, nearest_idx, step);

        if area.collide_point(next_p) || area.collide_line(base_point, next_p) {
            continue;
        }
        route.push((base_idx, next_p));
        if (next_p, end).length2() <= step_length * step_length && !area.collide_line(next_p, end) {
            break;
        }
    }
    let mut path = Vec::from([end]);
    let mut now_idx = route.len() - 1;
    loop {
        let (parent, point) = route[now_idx];
        path.push(point);
        if parent == now_idx {
            break;
        }
        now_idx = parent;
    }
    // path.push(start);
    path.reverse();
    path
}

trait AsMove<const L: usize> {
    type Config;
    fn new(config: &Self::Config) -> Self;
    fn normals(&self, pos: [f64; 3]) -> [[f64; 3]; L];
    fn apply(&self, pos: [f64; 3], sel_idx: usize, step: f64) -> [f64; 3];
}
struct XYZ;
impl AsMove<3> for XYZ {
    type Config = ();

    fn new(_: &Self::Config) -> Self {
        Self
    }

    fn normals(&self, _: [f64; 3]) -> [[f64; 3]; 3] {
        // [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
        [
            [1., 1., 0.].normal(),
            [0., 1., 1.].normal(),
            [1., 0., 2.].normal(),
        ]
    }

    fn apply(&self, pos: [f64; 3], sel_idx: usize, step: f64) -> [f64; 3] {
        let p = [
            [1., 1., 0.].normal(),
            [0., 1., 1.].normal(),
            [1., 0., 2.].normal(),
        ][sel_idx]
            .scale(step);
        pos.add(p)
    }
}
