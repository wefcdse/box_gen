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

pub mod lerp {
    pub trait Lerp: Copy {
        type Inner;
        type Float;
        fn lerp(self, delta: Self::Float) -> Self::Inner;
    }
    impl<T: ExtractInner2<f64>> Lerp for T {
        type Inner = f64;

        type Float = f64;

        fn lerp(self, delta: Self::Float) -> Self::Inner {
            let (a, b) = self.extract();
            a * (1. - delta) + b * delta
        }
    }
    trait ExtractInner2<Inner>: Copy {
        fn extract(self) -> (Inner, Inner);
    }
    impl<T: Into<f64> + Copy> ExtractInner2<f64> for (T, T) {
        fn extract(self) -> (f64, f64) {
            let (a, b) = self;
            (a.into(), b.into())
        }
    }
    impl<T: Into<f64> + Copy> ExtractInner2<f64> for [T; 2] {
        fn extract(self) -> (f64, f64) {
            let [a, b] = self;
            (a.into(), b.into())
        }
    }

    #[test]
    fn l() {
        assert_eq!((1i16, 0).lerp(0.5), 0.5);
        assert!(((1.3, -2.5).lerp(0.45) - -0.41) < 0.00000001);
    }
}
pub mod rempos {
    pub fn rempos(a: f64, b: f64) -> f64 {
        assert!(b > 0.);
        if a.is_sign_positive() {
            a % b
        } else {
            // a - (a / b).floor() * b
            a % b + b
        }
    }
    #[test]
    fn fp() {
        // dbg!(-1.2 % 0.51);
        let eq = |a, b| {
            assert!(rempos(a, b) > 0.);
            assert_eq!(rempos(a, b), a % b);
        };
        eq(1.412, 351.1);
        eq(1231.412, 351.1);
        let eqneg = |a, b| {
            assert!(rempos(a, b) > 0.);
            assert_eq!(rempos(a, b), a % b + b);
        };
        eqneg(-1.412, 351.1);
        eqneg(-1231.412, 351.1);
        eqneg(-1231.41432, 11.184838);
    }
}
