use cacl::lerp::Lerp;
use cacl::{gen_area, max_p, min_p};
use clap::Parser;
use obj::{Obj, Position};
use std::fs::{self, File};
use std::io::{BufReader, BufWriter, Write};
use std::path::PathBuf;
use support_type::Area;
use utils::index_xyz::*;
// const SPLIT: usize = 256;
// const OFFS: f64 = 20.;
pub mod cacl;
pub mod support_type;
pub mod utils;
pub mod vec2d;

#[derive(Parser, Debug)]
struct Args {
    input: PathBuf,
    split: usize,
    offs: f64,
    offs_high: f64,
    offs_low: f64,
}
fn main() {
    time!(all);
    let args = Args::parse();
    let input = BufReader::new(File::open(&args.input).unwrap());
    let mut output = {
        let f = fs::OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open("out.obj")
            .unwrap();
        BufWriter::new(f)
    };
    let mut output_txt = {
        let f = fs::OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open("out.txt")
            .unwrap();
        BufWriter::new(f)
    };
    let mut output_box = {
        let f = fs::OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open("out_box.txt")
            .unwrap();
        BufWriter::new(f)
    };

    time!(prase);
    let model: Obj<Position, usize> = obj::load_obj(input).unwrap();
    time!(prase, "prase");
    // dbg!(model.vertices[0]);
    // dbg!(&model.indices.len());

    time!(cf);
    let faces = model
        .indices
        .chunks(3)
        .map(|a| {
            assert!(a.len() == 3);
            let [x, y, z] = [a[0], a[1], a[2]];
            (x, y, z)
        })
        .collect::<Vec<_>>();
    time!(cf, "collect faces");
    // dbg!(faces.len());

    time!(mm);
    let (max, min) = model
        .vertices
        .iter()
        .copied()
        .fold((model.vertices[0], model.vertices[0]), |(ma, mi), i| {
            (max_p(ma, i), min_p(mi, i))
        });
    time!(mm, "max min");
    // dbg!(max, min);
    let split = args.split;

    let block_size = ((max.position[X] - min.position[X]).abs())
        .max((max.position[Y] - min.position[Y]).abs()) as f64
        / split as f64;
    let base_xy = [min.position[X] as f64, min.position[Y] as f64];

    let mut area = Area::new(
        split,
        base_xy,
        block_size,
        min.position[Z] as f64 - args.offs_low,
        max.position[Z] as f64 + args.offs_high,
    );
    // here
    gen_area(
        &mut area,
        &faces[..],
        &model.vertices[..],
        args.offs_high,
        args.offs,
        &max,
    );
    // a.height.iter_mut().for_each(|(_, a)| *a = 4.5);
    time!(write);
    area.write_to_obj(&mut output).unwrap();
    area.write_info(&mut output_txt).unwrap();
    area.write_to_box_x(&mut output_box).unwrap();
    area.write_to_box_y(&mut output_box).unwrap();
    area.write_to_box_z(&mut output_box).unwrap();
    area.write_to_box_width(&mut output_box).unwrap();
    area.write_to_box_height(&mut output_box).unwrap();
    time!(write, "write to file");
    time!(all, "total");

    {
        let mut output_collide = {
            let f = fs::OpenOptions::new()
                .write(true)
                .create(true)
                .truncate(true)
                .open("cld.obj")
                .unwrap();
            BufWriter::new(f)
        };
        let mut output_no_collide = {
            let f = fs::OpenOptions::new()
                .write(true)
                .create(true)
                .truncate(true)
                .open("no_cld.obj")
                .unwrap();
            BufWriter::new(f)
        };
        for _ in 0..50000 {
            let point = {
                let delta_p: [f64; 3] = rand::random();
                [
                    (min.position[X], max.position[X]).lerp(delta_p[X]),
                    (min.position[Y], max.position[Y]).lerp(delta_p[Y]),
                    (
                        min.position[Z] - args.offs_low as f32,
                        max.position[Z] + args.offs_high as f32,
                    )
                        .lerp(delta_p[Z]),
                ]
            };
            if area.collide_point(point) {
                writeln!(output_collide, "v {} {} {}", point[X], point[Y], point[Z]).unwrap();
            } else {
                writeln!(
                    output_no_collide,
                    "v {} {} {}",
                    point[X], point[Y], point[Z]
                )
                .unwrap();
            }
        }
    }
    {
        let mut output_collide = {
            let f = fs::OpenOptions::new()
                .write(true)
                .create(true)
                .truncate(true)
                .open("cld_line.obj")
                .unwrap();
            BufWriter::new(f)
        };
        let mut output_no_collide = {
            let f = fs::OpenOptions::new()
                .write(true)
                .create(true)
                .truncate(true)
                .open("no_cld_line.obj")
                .unwrap();
            BufWriter::new(f)
        };
        let mut vec_cld = Vec::new();
        let mut vec_no_cld = Vec::new();
        let mut counter_cld = 1;
        let mut counter_no_cld = 1;
        for _ in 0..1000000 {
            let point1 = {
                let delta_p: [f64; 3] = rand::random();
                [
                    (min.position[X], max.position[X]).lerp(delta_p[X]),
                    (min.position[Y], max.position[Y]).lerp(delta_p[Y]),
                    (
                        min.position[Z] - args.offs_low as f32,
                        max.position[Z] + args.offs_high as f32,
                    )
                        .lerp(delta_p[Z]),
                ]
            };
            let point2 = {
                let delta_p: [f64; 3] = rand::random();
                [
                    (min.position[X], max.position[X]).lerp(delta_p[X]),
                    (min.position[Y], max.position[Y]).lerp(delta_p[Y]),
                    (
                        min.position[Z] - args.offs_low as f32,
                        max.position[Z] + args.offs_high as f32,
                    )
                        .lerp(delta_p[Z]),
                ]
            };
            if area.collide_line(point1, point2) {
                writeln!(
                    output_collide,
                    "v {} {} {}",
                    point1[X], point1[Y], point1[Z]
                )
                .unwrap();
                writeln!(
                    output_collide,
                    "v {} {} {}",
                    point2[X], point2[Y], point2[Z]
                )
                .unwrap();
                vec_cld.push(counter_cld);
                counter_cld += 2;
            } else {
                writeln!(
                    output_no_collide,
                    "v {} {} {}",
                    point1[X], point1[Y], point1[Z]
                )
                .unwrap();
                writeln!(
                    output_no_collide,
                    "v {} {} {}",
                    point2[X], point2[Y], point2[Z]
                )
                .unwrap();
                vec_no_cld.push(counter_no_cld);
                counter_no_cld += 2;
            }
        }

        for i in vec_cld.iter() {
            writeln!(output_collide, "l {} {}", i, i + 1).unwrap();
        }
        for i in vec_no_cld.iter() {
            writeln!(output_no_collide, "l {} {}", i, i + 1).unwrap();
        }
    }
}
