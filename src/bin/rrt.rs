use core::f64;
use std::{f64::consts::PI, fs, io::BufWriter, isize, path::PathBuf};

use box_gen::{
    cacl::{
        jwd经纬度到xy,
        lerp::Lerp,
        point::{Point2Trait, PointTrait},
        到经纬度,
    },
    config::CONFIG,
    path_planning::{AsMove, Crane},
    support_type::Area,
    time,
    utils::{
        index_xyz::{self, Z},
        write_line_to_obj, write_line_to_obj_r90, Counter, Extend,
    },
};
use rand::random;
use rayon::iter::{IntoParallelIterator, ParallelIterator};
use std::io::Write;
use stupid_utils::select::DotSelect;

fn main() {
    time!(main);
    run_config();
    // run_scan_scene();
    time!(main, "main");
}
fn run_config() {
    let rad_to_deg = 180. / PI;
    let base = CONFIG.基准点经纬度高度.to_xyz();
    let 吊车回转中心水平坐标和变幅中心垂直坐标 = CONFIG
        .吊车回转中心水平经纬度和变幅中心高度
        .to_xyz()
        .sub(base);
    let 起始点 = CONFIG.起始点经纬度高度.to_xyz().sub(base);
    let 目标点 = CONFIG.目标点经纬度高度.to_xyz().sub(base);

    // let config = &(18.981, -1., [-15.6384, -10.2941, -11.8289]);
    let config = &(
        CONFIG.吊臂长度,
        CONFIG.变幅中心对回转中心偏移,
        吊车回转中心水平坐标和变幅中心垂直坐标,
    );
    let moveset = Crane::new(config);
    let (t0, t1, l) = dbg!(moveset.position_to_pose(起始点));
    dbg!(t1 * rad_to_deg);
    let area = Area::gen_from_obj_file(
        &CONFIG.文件名称,
        CONFIG.网格分割数量,
        CONFIG.上方偏移,
        CONFIG.下方偏移,
        CONFIG.偏移,
    );
    area.write_to_obj(&mut BufWriter::new(
        fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open("temp/rrt_area.obj")
            .unwrap(),
    ))
    .unwrap();
    write_line_to_obj(
        &mut BufWriter::new(
            fs::OpenOptions::new()
                .create(true)
                .write(true)
                .truncate(true)
                .open("temp/info_crane_start_end.obj")
                .unwrap(),
        ),
        [吊车回转中心水平坐标和变幅中心垂直坐标, 起始点, 目标点],
    )
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
        let (start, end) = (起始点, 目标点);
        // let (start, end) = (
        //     [-6.454714, -2.040601, 0.597431],
        //     [-2.407892, -6.342020, 0.597431],
        // );
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
        let counter = Counter::new();
        let paths = (0..CONFIG.rrt路径生成次数)
            .into_par_iter()
            .map(|_i| {
                let p = rrt_move::<3, Crane>(config, &area, start, end, CONFIG.rrt最大尝试采样次数);
                // dbg!(i);
                counter.count();
                if counter.show() % 10 == 0 {
                    println!("done {}", counter.show());
                }
                p
            })
            .filter_map(|p| p)
            .collect::<Vec<_>>();
        dbg!(paths.len());
        // best_path
        paths
            .into_iter()
            .min_by(|a, b| eval2(a, end).total_cmp(&eval2(b, end)))
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
        // dbg!(moveset.position_to_pose(CONFIG.起始点));
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
            .open("temp/rrt_move.csv")
            .unwrap();
        let rad_to_deg = 180. / PI;

        let (t01, t02, l) = moveset.position_to_pose(起始点);
        let 输出回转修正 = CONFIG.初始回转 - t01 * rad_to_deg;
        let 输出变幅修正 = CONFIG.初始变幅 - t02 * rad_to_deg;
        let 输出绳长修正 = CONFIG.初始绳长 - l;
        for p in v {
            let (t1, t2, l) = moveset.position_to_pose(p);
            writeln!(
                of,
                "{:.4},{:.4},{:.4}",
                t1 * rad_to_deg + 输出回转修正 + CONFIG.回转修正参数,
                t2 * rad_to_deg + 输出变幅修正,
                l + 输出绳长修正
            )
            .unwrap();
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
            writeln!(of, "{:.4} {:.4} {:.4}", t1, t2, l).unwrap();
        }
    }
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

    let step_length = area.block_width() * CONFIG.rrt步长对block倍数;
    assert!(step_length > 0.);
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
                let l = p1.sub(p).length2() * (2.03 - direction_fix);
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
                        CONFIG.rrt动作保持强度 * random::<f64>()
                    } else {
                        moveset
                            .neared(base_point, end, from_move_idx, step_length / 2.)
                            .select(CONFIG.rrt动作保持强度 * random::<f64>(), 0.)
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
        // let next_p = base_point.add(direction.scale(step_length));
        let next_step = step.signum() as isize * nearest_idx as isize;
        if !moveset.valid(area, next_p)
            || area.collide_point(next_p)
            || area.collide_line(base_point, next_p)
        {
            if !moveset.mercy(area, next_p, next_step) {
                continue;
            } else {
                // continue;
                // println!(
                //     "valid: {},{},{}",
                //     moveset.valid(area, next_p),
                //     moveset.mercy(area, next_p, next_step),
                //     next_step
                // );
            }
            // let len = route.len();
            // dbg!P
            //     len,
            //     next_p,
            //     moveset.valid(area, next_p),
            //     area.collide_point(next_p),
            //     area.collide_line(base_point, next_p)
            // );
            // println!(
            //     "p {},{}",
            //     area.collide_point(next_p),
            //     area.collide_line(base_point, next_p),
            //     next_step
            // );
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
            <= area.block_width()
                * area.block_width()
                * CONFIG.rrt终止点最大距离对block倍数
                * CONFIG.rrt终止点最大距离对block倍数
            && (next_p[Z] - end[Z]).abs() < CONFIG.rrt终止点最大距离对block倍数 * step_length
            && (!area.collide_line(next_p, end) || moveset.mercy(area, next_p, next_step) || false)
        // && ((next_p[Z] - end[Z]).abs() < step_length * 5.)
        // && next_p[Z] >= end[Z]
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
    let pos = path.last().unwrap().0;
    let mut fix: isize = 0;
    let mut last_move = 2;
    for (_, m, _) in path.iter().copied() {
        if last_move != m {
            fix += CONFIG.rrt动作切换惩罚系数 as isize;
        }
        last_move = m;
        if m == 2 {
            fix -= 3;
        }
    }
    (path.len() as isize + fix).clamp(0, isize::MAX) as usize
}
fn eval2(path: &[([f64; 3], usize, f64)], end: [f64; 3]) -> f64 {
    // let pos = path[path.len() - 2].0;
    // let l = pos.sub(end).length2() * 1000.;
    // dbg!(l);
    // return l as usize;
    let mut fix: f64 = 0.;
    let mut last_move: usize = 2;
    for (_, m, _) in path.iter().copied() {
        if last_move != m {
            fix += CONFIG.rrt动作切换惩罚系数;
        }
        last_move = m;
        if m == 2 {
            fix -= 0.7;
        }
    }
    dbg!(last_move);
    path.len() as f64 + fix + (last_move == 0).select(0., 0.)
}
