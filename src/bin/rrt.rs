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
    let path = loop {
        let (start, end) = loop {
            let s = area.random_point();
            let e = area.random_point();
            if !area.collide_point(s) && !area.collide_point(e) && area.collide_line(s, e) {
                break (s, e);
            }
        };
        dbg!(start, end);

        let path = rrt(&area, start, end);
        if path.len() > 5 {
            break path;
        }
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
