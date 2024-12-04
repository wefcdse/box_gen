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
        let wider = 2;
        for x in
            (min_idx[0].max(wider) - wider)..=(max_idx[0].min(area.data.x() - 1 - wider) + wider)
        {
            for y in (min_idx[1].max(wider) - wider)
                ..=(max_idx[1].min(area.data.y() - 1 - wider) + wider)
            {
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

    impl Lerp for ([f64; 2], [f64; 2]) {
        type Inner = [f64; 2];

        type Float = f64;

        fn lerp(self, delta: Self::Float) -> Self::Inner {
            let ([x1, y1], [x2, y2]) = self;
            [(x1, x2).lerp(delta), (y1, y2).lerp(delta)]
        }
    }
    impl Lerp for ([f64; 3], [f64; 3]) {
        type Inner = [f64; 3];

        type Float = f64;

        fn lerp(self, delta: Self::Float) -> Self::Inner {
            let ([x1, y1, z1], [x2, y2, z2]) = self;
            [
                (x1, x2).lerp(delta),
                (y1, y2).lerp(delta),
                (z1, z2).lerp(delta),
            ]
        }
    }
    trait ExtractInner2<Inner>: Copy {
        fn extract(self) -> (Inner, Inner);
    }

    macro_rules! impl_into {
        ($a:tt) => {
            impl ExtractInner2<f64> for ($a, $a) {
                fn extract(self) -> (f64, f64) {
                    let (a, b) = self;
                    (a.into(), b.into())
                }
            }
            impl ExtractInner2<f64> for [$a; 2] {
                fn extract(self) -> (f64, f64) {
                    let [a, b] = self;
                    (a.into(), b.into())
                }
            }
        };
    }
    impl_into!(i8);
    impl_into!(i16);
    impl_into!(i32);

    impl_into!(u8);
    impl_into!(u16);
    impl_into!(u32);
    // impl_into!(i64);
    // impl_into!(f128);
    impl_into!(f32);
    impl_into!(f64);

    // impl ExtractInner2<f64> for (i32, i32) {
    //     fn extract(self) -> (f64, f64) {
    //         let (a, b) = self;
    //         (a.into(), b.into())
    //     }
    // }
    // impl ExtractInner2<f64> for [i32; 2] {
    //     fn extract(self) -> (f64, f64) {
    //         let [a, b] = self;
    //         (a.into(), b.into())
    //     }
    // }

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

pub mod point {

    use crate::utils::index_xyz::{X, Y, Z};

    pub trait PointTrait {
        fn length(&self) -> f64 {
            self.length2().sqrt()
        }
        fn length2(&self) -> f64;
        fn sub(&self, rhs: Self) -> Self;
        fn add(&self, rhs: Self) -> Self;
        fn dot(&self, rhs: Self) -> f64;
        fn scale(&self, rhs: f64) -> Self;
        fn normal(&self) -> Self;
        fn loose_eq(&self, rhs: Self) -> bool;
        fn cross(&self, rhs: Self) -> Self;
    }
    impl PointTrait for [f64; 3] {
        fn length2(&self) -> f64 {
            self[X] * self[X] + self[Y] * self[Y] + self[Z] * self[Z]
        }

        fn sub(&self, rhs: Self) -> Self {
            [(self[X] - rhs[X]), (self[Y] - rhs[Y]), (self[Z] - rhs[Z])]
        }

        fn add(&self, rhs: Self) -> Self {
            [(self[X] + rhs[X]), (self[Y] + rhs[Y]), (self[Z] + rhs[Z])]
        }

        fn dot(&self, rhs: Self) -> f64 {
            self[X] * rhs[X] + self[Y] * rhs[Y] + self[Z] * rhs[Z]
        }

        fn normal(&self) -> Self {
            [
                self[X] / self.length(),
                self[Y] / self.length(),
                self[Z] / self.length(),
            ]
        }

        fn scale(&self, rhs: f64) -> Self {
            [self[X] * rhs, self[Y] * rhs, self[Z] * rhs]
        }

        fn loose_eq(&self, rhs: Self) -> bool {
            let b = 1e-10;
            (self[X] - rhs[X]).abs() <= b
                && (self[Y] - rhs[Y]).abs() <= b
                && (self[Z] - rhs[Z]).abs() <= b
        }

        fn cross(&self, rhs: Self) -> Self {
            [
                self[1] * rhs[2] - self[2] * rhs[1],
                self[2] * rhs[0] - self[0] * rhs[2],
                self[0] * rhs[1] - self[1] * rhs[0],
            ]
        }
    }
    pub trait Point2Trait {
        fn length(&self) -> f64 {
            self.length2().sqrt()
        }
        fn length2(&self) -> f64;
    }
    impl Point2Trait for ([f64; 3], [f64; 3]) {
        fn length2(&self) -> f64 {
            self.0.sub(self.1).length2()
        }
    }
    impl Point2Trait for [[f64; 3]; 2] {
        fn length2(&self) -> f64 {
            self[0].sub(self[1]).length2()
        }
    }
    #[test]
    fn t() {
        use rand::random;
        assert_eq!([1., 1., 1.,].sub([0.3, 0.2, -0.6]), [0.7, 0.8, 1.6]);
        dbg!([1., 1., 1.,].dot([0.3, 0.2, 0.6]),);
        dbg!([213., 413., 13.].normal().length());

        let test = [
            (
                [0.6952728948550719, 0.8856387014784037, 0.07952342062781792],
                [0.7684203493004392, 0.5531224343830885, 0.36618075592317245],
                [0.2803176611740497, -0.19348813955452, -0.2959717641812078],
            ),
            (
                [0.6026368171302262, -0.491278785842444, 0.22342359116365107],
                [0.18030142487852108, -0.4911784234672233, 0.847900458499978],
                [-0.3068146604939731, -0.470692441715403, -0.207423936661347],
            ),
        ];
        for (a, b, r) in test {
            // dbg!(a.cross(b));
            assert!(a.cross(b).loose_eq(r));
        }
        dbg!(random::<[f64; 3]>());
        dbg!(random::<[f64; 3]>());
    }
}
