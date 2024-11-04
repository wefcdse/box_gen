use obj::Position;

use crate::{support_type::Area, time};

pub fn min_p(a: Position, b: Position) -> Position {
    Position {
        position: [
            a.position[0].min(b.position[0]),
            a.position[1].min(b.position[1]),
            a.position[2].min(b.position[2]),
        ],
    }
}
pub fn max_p(a: Position, b: Position) -> Position {
    Position {
        position: [
            a.position[0].max(b.position[0]),
            a.position[1].max(b.position[1]),
            a.position[2].max(b.position[2]),
        ],
    }
}

pub fn gen_area(
    area: &mut Area,
    faces: &[(usize, usize, usize)],
    vertices: &[Position],
    offs_high: f64,
    offs: f64,
    max: &Position,
) {
    time!(fa);
    for (idx, (a, b, c)) in faces.iter().copied().enumerate() {
        let min = min_p(min_p(vertices[a], vertices[b]), vertices[c]);
        let max = max_p(max_p(vertices[a], vertices[b]), vertices[c]);
        let min_xy = [min.position[0] as f64, min.position[1] as f64];
        let max_xy = [max.position[0] as f64, max.position[1] as f64];

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
                let max = max_p(max_p(vertices[a], vertices[b]), vertices[c]);
                *height = height.max(max.position[2] as f64);
            }
        });
    time!(gen, "gen height");
    area.data.iter_mut().for_each(|(_, (_, height))| {
        if *height == f64::NEG_INFINITY {
            *height = max.position[2] as f64 + offs_high;
        } else {
            *height += offs;
        }
    });
}
