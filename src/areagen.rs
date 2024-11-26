use cacl::{gen_area, max_p, min_p};
use obj::{Obj, Position};
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use support_type::Area;
use utils::index_xyz::*;

use crate::{cacl, support_type, time, utils};

impl Area {
    pub fn gen_from_obj_file(
        file: impl AsRef<Path>,
        split: usize,
        offs_high: f64,
        offs_low: f64,
        offs: f64,
    ) -> Self {
        let input = BufReader::new(File::open(file).unwrap());
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
        // let split = split;

        let block_size = ((max.position[X] - min.position[X]).abs())
            .max((max.position[Y] - min.position[Y]).abs()) as f64
            / split as f64;
        let base_xy = [min.position[X] as f64, min.position[Y] as f64];

        let mut area = Area::new(
            split,
            base_xy,
            block_size,
            min.position[Z] as f64 - offs_low,
            max.position[Z] as f64 + offs_high,
        );
        // here
        gen_area(
            &mut area,
            &faces[..],
            &model.vertices[..],
            offs_high,
            offs,
            &max,
        );
        // dbg!(faces.len());
        area
    }
}
