use core::f64;
use std::{fs, io::BufWriter};

use box_gen::{cacl::point::PointTrait, support_type::Area, utils::write_line_to_obj};

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
    let step_length = area.block_width();
    let mut path = Vec::from([start]);
    let mut route = Vec::from([(0, start)]);
    loop {
        let nearest_idx = |p| {
            let mut nearest_idx = 0;
            let mut l2 = f64::MAX;
            for (idx, (_, p1)) in route.iter().copied().enumerate() {
                let l = p1.sub(p).length2();
                if l < l2 {
                    l2 = l;
                    nearest_idx = idx;
                }
            }
            nearest_idx
        };

        let p = area.random_point();
        if area.collide_point(p) || area.collide_line(*path.last().unwrap(), p) {
            continue;
        }
        path.push(p);
        if !area.collide_line(p, end) {
            break;
        }
    }
    path.push(end);
    path
}
