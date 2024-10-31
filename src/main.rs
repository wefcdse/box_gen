use clap::Parser;
use obj::{Obj, Position};
use std::fs::{self, File};
use std::io::{BufReader, BufWriter};
use std::path::PathBuf;
use support_type::Area;
// const SPLIT: usize = 256;
// const OFFS: f32 = 20.;
pub mod support_type;
pub mod utils;
pub mod vec2d;
#[derive(Parser, Debug)]
struct Args {
    input: PathBuf,
    split: usize,
    offs: f32,
    offs_high: f32,
    offs_low: f32,
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
        .max((max.position[1] - min.position[1]).abs())
        / split as f32;
    let base_xy = [min.position[0], min.position[1]];

    let mut area = Area::new(
        split,
        base_xy,
        block_size,
        min.position[2] - args.offs_low,
        max.position[2] + args.offs_high,
    );
    time!(fa);
    for (idx, (a, b, c)) in faces.iter().copied().enumerate() {
        let min = min_p(
            min_p(model.vertices[a], model.vertices[b]),
            model.vertices[c],
        );
        let max = max_p(
            max_p(model.vertices[a], model.vertices[b]),
            model.vertices[c],
        );
        let min_xy = [min.position[0], min.position[1]];
        let max_xy = [max.position[0], max.position[1]];

        let min_idx = area.in_block(min_xy);
        let max_idx = area.in_block(max_xy);
        assert!(min_idx[0] <= max_idx[0]);
        assert!(min_idx[1] <= max_idx[1]);

        for x in min_idx[0]..=max_idx[0] {
            for y in min_idx[1]..=max_idx[1] {
                area.data[[x, y]].0.push(idx);
            }
        }
    }
    time!(fa, "face to area");
    time!(gen);
    area.data
        .iter_mut()
        .for_each(|(_, (contained_faces, height))| {
            for f in contained_faces.iter().copied() {
                let (a, b, c) = faces[f];
                let max = max_p(
                    max_p(model.vertices[a], model.vertices[b]),
                    model.vertices[c],
                );
                *height = height.max(max.position[2]);
            }
        });
    time!(gen, "gen height");
    area.data.iter_mut().for_each(|(_, (_, height))| {
        if *height == f32::NEG_INFINITY {
            *height = max.position[2] + args.offs_high;
        } else {
            *height += args.offs;
        }
    });
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

fn min_p(a: Position, b: Position) -> Position {
    Position {
        position: [
            a.position[0].min(b.position[0]),
            a.position[1].min(b.position[1]),
            a.position[2].min(b.position[2]),
        ],
    }
}
fn max_p(a: Position, b: Position) -> Position {
    Position {
        position: [
            a.position[0].max(b.position[0]),
            a.position[1].max(b.position[1]),
            a.position[2].max(b.position[2]),
        ],
    }
}
