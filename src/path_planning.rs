use std::ops::Sub;

use crate::{cacl::point::PointTrait, disable};

pub trait AsMove<const L: usize> {
    type Config;
    fn new(config: &Self::Config) -> Self;
    fn normals(&self, pos: [f64; 3]) -> [[f64; 3]; L];
    fn apply(&self, pos: [f64; 3], sel_idx: usize, step: f64) -> [f64; 3];
    fn default_move(&self) -> usize;
}
#[allow(unused, clippy::upper_case_acronyms)]
struct XYZ;
impl AsMove<3> for XYZ {
    type Config = ();

    fn new(_: &Self::Config) -> Self {
        Self
    }

    fn normals(&self, _: [f64; 3]) -> [[f64; 3]; 3] {
        // [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
        // [
        //     [1., 1., 0.].normal(),
        //     [0., 1., 1.].normal(),
        //     [1., 0., 2.].normal(),
        // ]
        [
            [0.09275263006051904, 0.5062427205635203, 0.08394260037843648].normal(),
            [0.505252002327924, 0.0036806377496515497, 0.6857313789597959].normal(),
            [0.7530463184535672, 0.3125142326453717, 0.05447322306410152].normal(),
        ]
    }

    fn apply(&self, pos: [f64; 3], sel_idx: usize, step: f64) -> [f64; 3] {
        let p = self.normals(pos)[sel_idx].scale(step);
        pos.add(p)
    }

    fn default_move(&self) -> usize {
        0
    }
}
#[test]
fn d() {
    use rand::random;

    println!("{:?}.normal(),", random::<[f64; 3]>());
    println!("{:?}.normal(),", random::<[f64; 3]>());
    println!("{:?}.normal(),", random::<[f64; 3]>());
}

pub struct Crane {
    l: f64,
    yaw_offs: f64,
    base: [f64; 3],
}

impl Crane {
    pub fn position_to_pose(&self, pos: [f64; 3]) -> (f64, f64, f64) {
        // y
        //
        // 0    x
        let [x, y, z] = pos.sub(self.base);
        disable!(pos);
        let theta1 = {
            // // let t =
            // let xp = x > 0.;
            // let yp = y > 0.;
            y.atan2(x)
        };

        let xy = (x * x + y * y).sqrt().sub(self.yaw_offs);

        let theta2 = (xy / self.l).acos();

        let l = (self.l * self.l - xy * xy).sqrt() - z;
        (theta1, theta2, l)
    }
    pub fn pose_to_position(&self, (theta1, theta2, l): (f64, f64, f64)) -> [f64; 3] {
        let z = self.l * theta2.sin() - l;
        let xy = self.l * theta2.cos() + self.yaw_offs;
        let x = xy * theta1.cos();
        let y = xy * theta1.sin();
        [x, y, z].add(self.base)
    }
}
#[test]
fn a() {
    use crate::utils::index_xyz::*;
    let d = Crane {
        l: 40.,
        yaw_offs: 60.,
        base: [0., 0., 100.],
    };
    let p = [70., 70., -12.];
    assert!([p[X] - d.base[X], p[Y] - d.base[Y], 0.].length() < (d.l + d.yaw_offs));
    let mut a = dbg!(d.position_to_pose(p));
    let b = dbg!(d.pose_to_position(a));
    assert!(p.loose_eq(b));

    let mut point_vec = Vec::new();
    point_vec.push(d.base);
    point_vec.push(d.base.add([0., 0., d.l]));
    point_vec.push(p);
    for _ in 0..100 {
        a.0 += std::f64::consts::PI / 180. * 3.;
        point_vec.push(d.pose_to_position(a));
    }
    for _ in 0..80 {
        a.1 += std::f64::consts::PI / 180. * 3.;
        point_vec.push(d.pose_to_position(a));
    }
    for _ in 0..100 {
        a.0 += std::f64::consts::PI / 180. * 1.;
        point_vec.push(d.pose_to_position(a));
    }
    for _ in 0..30 {
        a.2 += 1.;
        point_vec.push(d.pose_to_position(a));
    }

    for _ in 0..100 {
        a.1 += std::f64::consts::PI / 180. * 3.;
        point_vec.push(d.pose_to_position(a));
    }
    use std::fs;
    use std::io::BufWriter;
    crate::utils::write_line_to_obj(
        &mut BufWriter::new(
            fs::OpenOptions::new()
                .create(true)
                .write(true)
                .truncate(true)
                .open("temp/cyc.obj")
                .unwrap(),
        ),
        point_vec.iter().copied(),
    )
    .unwrap();
}
impl AsMove<3> for Crane {
    type Config = (f64, f64, [f64; 3]);

    fn new(config: &Self::Config) -> Self {
        Self {
            l: config.0,
            yaw_offs: config.1,
            base: config.2,
        }
    }

    fn normals(&self, pos: [f64; 3]) -> [[f64; 3]; 3] {
        let [x, y, z] = pos.sub(self.base);
        let t1pos_normal = [0., 0., 1.].cross([x, y, 0.]).normal();
        let t2pos_normal = [x, y, z].cross(t1pos_normal).normal();
        [t1pos_normal, t2pos_normal, [0., 0., -1.]]
    }

    fn apply(&self, pos: [f64; 3], sel_idx: usize, step: f64) -> [f64; 3] {
        // const PI: f64 = std::f64::consts::PI;
        let [x, y, _] = pos.sub(self.base);
        let step_rad1 = step / (x * x + y * y).sqrt();
        let step_rad2 = step / self.l;
        let (t1, t2, l) = self.position_to_pose(pos);
        let pose1 = match sel_idx {
            0 => (t1 + step_rad1, t2, l),
            1 => (t1, t2 + step_rad2, l),
            2 => (t1, t2, l + step),
            _ => unreachable!(),
        };
        self.pose_to_position(pose1)
    }

    fn default_move(&self) -> usize {
        2
    }
}