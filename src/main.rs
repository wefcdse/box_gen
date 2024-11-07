use cacl::{gen_area, max_p, min_p};
use clap::Parser;
use obj::{Obj, Position};
use std::fs::{self, File};
use std::io::{BufReader, BufWriter};
use std::path::PathBuf;
use support_type::Area;
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

    let block_size = ((max.position[0] - min.position[0]).abs())
        .max((max.position[1] - min.position[1]).abs()) as f64
        / split as f64;
    let base_xy = [min.position[0] as f64, min.position[1] as f64];

    let mut area = Area::new(
        split,
        base_xy,
        block_size,
        min.position[2] as f64 - args.offs_low,
        max.position[2] as f64 + args.offs_high,
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
}
