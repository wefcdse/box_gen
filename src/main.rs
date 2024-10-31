use obj::{load_obj, Obj, Position};
use std::fs::File;
use std::hint::black_box;
use std::io::BufReader;
use std::time::Instant;
const SPLIT: usize = 8;
mod support_type;
pub mod vec2d;
fn main() {
    let input = BufReader::new(File::open("./Block.obj").unwrap());
    let t = Instant::now();
    let model: Obj<Position, usize> = obj::load_obj(input).unwrap();
    dbg!(t.elapsed());
    // Do whatever you want
    // model.vertices;
    dbg!(model.vertices[0]);
    // model.indices;
    dbg!(&model.indices.len());
    let t = Instant::now();
    let faces = model
        .indices
        .chunks(3)
        .map(|a| {
            assert!(a.len() == 3);
            let [x, y, z] = [a[0], a[1], a[2]];
            (x, y, z)
        })
        .collect::<Vec<_>>();
    dbg!(t.elapsed());
    dbg!(faces.len());
    let (max, min) = model
        .vertices
        .iter()
        .copied()
        .fold((model.vertices[0], model.vertices[0]), |(ma, mi), i| {
            (max(ma, i), min(mi, i))
        });
    dbg!(max, min);
    let split = SPLIT;
}

fn min(a: Position, b: Position) -> Position {
    Position {
        position: [
            a.position[0].min(b.position[0]),
            a.position[1].min(b.position[1]),
            a.position[2].min(b.position[2]),
        ],
    }
}
fn max(a: Position, b: Position) -> Position {
    Position {
        position: [
            a.position[0].max(b.position[0]),
            a.position[1].max(b.position[1]),
            a.position[2].max(b.position[2]),
        ],
    }
}
