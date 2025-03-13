use core::f64;
use std::{f64::consts::PI, fs, io::BufWriter, path::PathBuf};

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
                t1 * rad_to_deg + 输出回转修正,
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

fn run_scan_scene20241210() {
    // let config = &(18.981, -1., [-15.6384, -10.2941, -11.8289]);
    let config = &(5.15744, -0.252292, [2.493617, -1.038677, 1.021000]);

    let area = Area::gen_from_obj_file(
        r#"C:\Users\yaoyj\Desktop\2024-12-10\env.obj"#,
        300,
        20.,
        20.,
        0.02,
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
        let (start, end) = (
            [7.011840, -2.013177, 0.397000],
            [7.107685, 0.592520, 0.928000],
        );
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

        let paths = (0..1024)
            .into_par_iter()
            .map(|i| {
                let p = rrt_move::<2, Crane>(config, &area, start, end, 5000);
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
            .open("temp/rrt_move.csv")
            .unwrap();
        let rad_to_deg = 180. / PI;
        for p in v {
            let (t1, t2, l) = moveset.position_to_pose(p);
            writeln!(of, "{:.4},{:.4},{:.4}", t1 * rad_to_deg, t2 * rad_to_deg, l).unwrap();
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

fn run_scan_scene20241204() {
    // let config = &(18.981, -1., [-15.6384, -10.2941, -11.8289]);
    let config = &(18.981, -1., [0., 0., 0.]);
    let config = &(18.981, -1., [-10.294100, 15.638400, -11.828900]);

    let area = Area::gen_from_obj_file(
        r#"C:\Users\yaoyj\Desktop\2024-12-04\env.obj"#,
        300,
        20.,
        20.,
        0.02,
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
        let (start, end) = (
            [-1.92, -12.610593, -13.373562],
            [-2.8623, -4.74634, -13.373562],
        );
        let (start, end) = (
            [-13.136984, -0.590593, -13.476562],
            [-3.265594, 0.447873, -13.476562],
        );
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

        let paths = (0..4096)
            .into_par_iter()
            .map(|i| {
                let p = rrt_move::<2, Crane>(config, &area, start, end, 2500);
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
            .open("temp/rrt_move.csv")
            .unwrap();
        let rad_to_deg = 180. / PI;
        for p in v {
            let (t1, t2, l) = moveset.position_to_pose(p);
            writeln!(of, "{},{},{}", t1 * rad_to_deg, t2 * rad_to_deg, l).unwrap();
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

fn run_sim_scene() {
    let config = &(10., 0.5, [0., 0., 2.]);
    let area = Area::gen_from_obj_file("s2.obj", 300, 20., 20., 0.02);
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
        let (start, end) = (
            [-6.454811, -4.306059, 0.597431],
            [-0.449157, -4.599950, 0.597431],
        );
        let (start, end) = (
            [-6.454714, -2.040601, 0.597431],
            [-2.407892, -6.342020, 0.597431],
        );
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

        let paths = (0..2048)
            .into_par_iter()
            .map(|i| {
                let p = rrt_move::<3, Crane>(config, &area, start, end, 2_0000);
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

fn run_scan_scene() {
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
        // let next_p = base_point.add(direction.scale(step_length));

        if !moveset.valid(area, next_p)
            || area.collide_point(next_p)
            || area.collide_line(base_point, next_p)
        {
            // let len = route.len();
            // dbg!(
            //     len,
            //     next_p,
            //     moveset.valid(area, next_p),
            //     area.collide_point(next_p),
            //     area.collide_line(base_point, next_p)
            // );
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
        if ([next_p[X], next_p[Y], next_p[Z]], [end[X], end[Y], end[Z]]).length2()
            <= step_length
                * step_length
                * CONFIG.rrt终止点最大距离对block倍数
                * CONFIG.rrt终止点最大距离对block倍数
            && !area.collide_line(next_p, end)
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
    let mut fix = 0;
    let mut last_move = path[0].1;
    for (_, m, _) in path.iter().copied() {
        if last_move != m {
            fix += CONFIG.rrt动作切换惩罚系数;
        }
        last_move = m;
    }
    path.len() + fix
}

#[test]
fn t() {
    let base = [370700.255, 4304529.799, 46.848];
    use box_gen::cacl::point::PointTrait;

    let a吊钩旋转中心 = [370702.4682042303, 4304536.876396358, 49.265];
    dbg!(a吊钩旋转中心.sub(base));
    let a变幅中心 = [370701.4836496849, 4304532.010081603, 47.869];
    dbg!(a变幅中心.sub(base));
    let start = [370702.2681766035, 4304536.810839642, 47.245];
    dbg!(start.sub(base));
    let end = [370699.6624795006, 4304536.906685061, 47.776];
    dbg!(end.sub(base));
}
#[cfg(test)]
mod t {
    use std::fs;

    use box_gen::cacl::jwd经纬度到xy;

    #[test]
    fn t1() {
        {
            // 基准
            let a = 121.50947236;
            let b = 38.88067769;
            let u = 48.64;
        }
        {
            // 中心1
            let a = 121.509485082;
            let b = 38.880735289;
            let u = 48.437;
        }
        {
            // 中心2
            let a = 121.509444115;
            let b = 38.880741065;
            let u = 48.29;
        }
        {
            // 起始点
            let a = 121.509479776;
            let b = 38.880693037;
            let u = 48.643;
        }
        {
            // 目标点
            let a = 121.509446066;
            let b = 38.880693192;
            let u = 48.181;
        }
        // 起始点2
        let a = 121.509480959;
        let b = 38.880691444;
        let u = 48.056;

        let basex = 370712.9724808662;
        let basey = 4304591.115754284;
        let basez = 47.877;

        let [x, y] = jwd经纬度到xy(a, b);
        dbg!((x, y));
        let (x, y, z) = dbg!((x - basex, y - basey, u - basez));
        fs::write("config.obj", format!("v {} {} {}", x, y, z)).unwrap();
        // let (x1, y1) = (370702.4682042303, 4304536.876396358);
        // dbg!((x - x1).abs() < 0.001, (y - y1).abs() < 0.001);
    }
}
