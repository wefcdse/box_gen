use core::f64;
use std::{fs, io::BufWriter};

use box_gen::{
    cacl::{
        lerp::Lerp,
        point::{Point2Trait, PointTrait},
    },
    path_planning::{AsMove, Crane},
    support_type::Area,
    utils::{
        index_xyz::{self, Z},
        write_line_to_obj,
    },
};
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use std::io::Write;
fn main() {
    let config = &(300., 20., [0., 0., 0.]);
    let area = Area::gen_from_obj_file("Block.obj", 150, 20., 20., 0.1);
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
            let s = area.random_point().scale(0.8);
            let e = area.random_point().scale(0.8);
            if s[Z].max(e[Z]) > area.max()[Z] - 30. {
                continue;
            }
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

        let paths = (0..256)
            .into_par_iter()
            .map(|i| {
                let p = rrt_move::<3, Crane>(config, &area, start, end, 1_0000);
                dbg!(i);
                p
            })
            .filter_map(|p| p)
            .collect::<Vec<_>>();
        dbg!(paths.len());
        // best_path
        paths
            .into_iter()
            .min_by(|a, b| eval(a).cmp(&eval(b)))
            .unwrap_or(Vec::from([(start, 0, 0.), (end, 0, 0.)]))
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
        path.iter().map(|(a, _, _)| *a),
    )
    .unwrap();
    {
        let moveset = Crane::new(config);
        let mut v = Vec::new();
        let mut last_pos = path[0].0;
        v.push(last_pos);
        let mut last_m = path[0].1;
        for (i, (pos, m, _)) in path.iter().copied().enumerate() {
            if m != last_m || i == path.len() - 1 {
                v.push(last_pos);
            }
            last_pos = pos;
            last_m = m;
        }

        let mut of = fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open("temp/rrt_move.obj")
            .unwrap();

        for p in v {
            let (t1, t2, l) = moveset.position_to_pose(p);
            writeln!(of, "{} {} {}", t1, t2, l).unwrap();
        }
    }

    {
        let moveset = Crane::new(config);
        let mut of = fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open("temp/rrt_move_all.obj")
            .unwrap();

        for (p, _, _) in path.iter() {
            let (t1, t2, l) = moveset.position_to_pose(*p);
            writeln!(of, "{} {} {}", t1, t2, l).unwrap();
        }
    }
}
#[allow(dead_code)]
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
    max_count: usize,
) -> Option<Vec<([f64; 3], usize, f64)>> {
    struct Node {
        pos: [f64; 3],
        root: usize,
        from_move_idx: usize,
        #[allow(unused)]
        from_move_step: f64,
    }
    let moveset = M::new(config);

    let step_length = area.block_width() * 3.0;
    let mut route = Vec::from([Node {
        pos: start,
        root: 0,
        from_move_idx: moveset.default_move(),
        from_move_step: 0.0,
    }]);
    let mut counter = 0;
    loop {
        let nearest_idx = |p: [f64; 3]| {
            let mut nearest_idx = 0;
            let mut l2 = f64::MAX;
            for (idx, p1) in route.iter().map(|e| e.pos).enumerate() {
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
        let (base_point, base_idx, from_move_idx) = {
            let ni = nearest_idx(p);
            (route[ni].pos, ni, route[ni].from_move_idx)
        };

        let direction = p.sub(base_point).normal();
        let moves = moveset.normals(base_point);
        // 选择最接近的方向
        let nearest_idx = moves
            .into_iter()
            .enumerate()
            .fold((f64::MIN, 0), |(max_abs, max_idx), (idx, normal)| {
                let abs = normal.dot(direction).abs()
                    + if idx == from_move_idx {
                        // random::<f64>() * 0.9
                        // 1. * random::<f64>()
                        0.9
                    } else {
                        0.0
                    };
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
        // route.push((base_idx, next_p));
        route.push(Node {
            pos: next_p,
            root: base_idx,
            from_move_idx: nearest_idx,
            from_move_step: step,
        });
        counter += 1;
        use index_xyz::*;
        if ([next_p[X], next_p[Y], 0.], [end[X], end[Y], 0.]).length2()
            <= step_length * step_length * 4.
            && !area.collide_line(next_p, end)
        {
            break;
        }
        if counter > max_count {
            return None;
        }
    }
    dbg!(counter);
    let mut path = Vec::from([(end, 0, 0.)]);
    let mut now_idx = route.len() - 1;
    loop {
        let Node {
            root: parent,
            pos: point,
            from_move_idx,
            from_move_step,
            ..
        } = route[now_idx];
        path.push((point, from_move_idx, from_move_step));
        if parent == now_idx {
            break;
        }
        now_idx = parent;
    }
    // path.push(start);
    path.reverse();
    Some(path)
}

fn eval(path: &[([f64; 3], usize, f64)]) -> usize {
    let mut fix = 0;
    let mut last_move = path[0].1;
    for (_, m, _) in path.iter().copied() {
        if last_move != m {
            fix += 10;
        }
        last_move = m;
    }
    path.len() + fix
}
