use std::{
    fmt::Debug,
    io::{self, Write},
};

use smallvec::SmallVec;

use crate::{
    cacl::{lerp::Lerp, point::PointTrait, rempos::rempos},
    disable,
    utils::index_xyz::*,
    vec2d::Vec2d,
};

mod utils {
    use crate::utils::random_point_in;

    use super::Area;

    impl Area {
        pub fn random_point(&self) -> [f64; 3] {
            random_point_in(self.min(), self.max())
        }
    }
}

// 0####1@@@@
// #####@@@@@
// #####@@@@@
// 1$$$$
// $$$$$
// $$$$$
// 2
#[non_exhaustive]
pub struct Area {
    pub(crate) data: Vec2d<(SmallVec<[usize; 4]>, f64)>,
    // pub height: Vec2d<f64>,
    // a: SmallVec<[usize; 15]>,
    base: [f64; 2],
    zmin: f64,
    #[allow(dead_code)]
    zmax: f64,
    block_width: f64,
}
impl Debug for Area {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Area")
            .field("base", &self.base)
            .field("zmin", &self.zmin)
            .field("zmax", &self.zmax)
            .field("block_width", &self.block_width)
            .finish_non_exhaustive()
    }
}
impl Area {
    pub fn new(l: usize, base: [f64; 2], block_width: f64, zmin: f64, zmax: f64) -> Self {
        Self {
            data: Vec2d::new_filled(l, l, (SmallVec::new(), f64::NEG_INFINITY)),
            base,
            block_width,
            zmin,
            zmax,
        }
    }
    pub fn in_block(&self, pos: [f64; 2]) -> [usize; 2] {
        let dx = pos[X] - self.base[X];
        let dy = pos[Y] - self.base[Y];
        if !(dx >= 0. && dy >= 0.) {
            dbg!();
            panic!("dx {}, dy {}, pos {:?}", dx, dy, pos);
        }
        assert!(dx >= 0.);
        assert!(dy >= 0.);
        [
            ((dx / self.block_width).floor() as usize)
                .max(0)
                .min(self.data.x() - 1),
            ((dy / self.block_width).floor() as usize)
                .max(0)
                .min(self.data.y() - 1),
        ]
    }

    pub fn in_area(&self, pos: [f64; 2]) -> bool {
        pos[X] >= self.base[X]
            && pos[X] <= self.base[X] + self.block_width * self.data.x() as f64
            && pos[Y] >= self.base[Y]
            && pos[Y] <= self.base[Y] + self.block_width * self.data.y() as f64
    }
    pub fn pos(&self, block: [usize; 2]) -> [f64; 2] {
        [
            (block[X] as f64 * self.block_width + self.base[X]),
            (block[Y] as f64 * self.block_width + self.base[Y]),
        ]
    }
    pub fn height(&self, block: [usize; 2]) -> f64 {
        self.data[block].1
    }
    pub fn write_to_obj<W: Write>(&self, file: &mut W) -> io::Result<()> {
        for ((x, y), z) in self.data.iter().map(|(idx, (_, h))| (idx, *h)) {
            let [x, y] = self.pos([x, y]);
            writeln!(
                file,
                "v {} {} {}",
                (x + 0.5 * self.block_width),
                (y + 0.5 * self.block_width),
                z,
            )?;
        }
        Ok(())
    }
    pub fn max(&self) -> [f64; 3] {
        [
            self.base[X] + self.block_width * self.data.x() as f64,
            self.base[Y] + self.block_width * self.data.y() as f64,
            self.zmax,
        ]
    }
    pub fn min(&self) -> [f64; 3] {
        [self.base[X], self.base[Y], self.zmin]
    }
    pub fn block_width(&self) -> f64 {
        self.block_width
    }
}
impl Area {
    pub fn write_info<W: Write>(&self, file: &mut W) -> io::Result<()> {
        writeln!(file, "lowest {}", self.zmin)?;
        writeln!(file, "base xy {:?}", self.base)?;
        writeln!(file, "block_size {}", self.block_width)?;
        Ok(())
    }
    pub fn write_to_box_x<W: Write>(&self, file: &mut W) -> io::Result<()> {
        for ((x, y), _) in self.data.iter().map(|(idx, (_, h))| (idx, *h)) {
            let [x, _] = self.pos([x, y]);
            write!(
                file,
                "{} ",
                (x + 0.5 * self.block_width) - self.block_width * 0.5
            )?;
        }
        writeln!(file)?;
        Ok(())
    }
    pub fn write_to_box_y<W: Write>(&self, file: &mut W) -> io::Result<()> {
        for ((x, y), _) in self.data.iter().map(|(idx, (_, h))| (idx, *h)) {
            let [_, y] = self.pos([x, y]);
            write!(
                file,
                "{} ",
                -(y + 0.5 * self.block_width) - self.block_width * 0.5
            )?;
        }
        writeln!(file)?;
        Ok(())
    }
    pub fn write_to_box_z<W: Write>(&self, file: &mut W) -> io::Result<()> {
        for (_, _z) in self.data.iter().map(|(idx, (_, h))| (idx, *h)) {
            write!(file, "{} ", self.zmin)?;
        }
        writeln!(file)?;
        Ok(())
    }
    pub fn write_to_box_width<W: Write>(&self, file: &mut W) -> io::Result<()> {
        for _ in self.data.iter() {
            write!(file, "{} ", self.block_width)?;
        }
        writeln!(file)?;
        Ok(())
    }
    pub fn write_to_box_height<W: Write>(&self, file: &mut W) -> io::Result<()> {
        for (_, z) in self.data.iter().map(|(idx, (_, h))| (idx, *h)) {
            write!(file, "{} ", (z - self.zmin))?;
        }
        writeln!(file)?;
        Ok(())
    }
}
// #[test]
// fn write() {
//     let mut f = std::fs::OpenOptions::new()
//         .write(true)
//         .create(true)
//         .truncate(true)
//         .open("exp.obj")
//         .unwrap();
//     let mut a1 = Area::new(5, [0., 0.], 2., 0., 0.);
//     a1.data.iter_mut().for_each(|(_, (_, a))| *a = 4.5);
//     a1.write_to_obj(&mut f).unwrap();
// }

// phy
#[allow(unused)]
impl Area {
    pub fn collide_point(&self, pos: [f64; 3]) -> bool {
        let [x, y, z] = pos;
        disable!(pos);
        if !self.in_area([x, y]) {
            return true;
        }
        let block = self.in_block([x, y]);
        let height = self.height(block);
        height >= z
    }
    pub fn collide_area(&self, pos: [f64; 2], height: f64, r: f64) -> bool {
        let ([x, y], z) = (pos, height);
        disable!(pos, height);
        let block_min = [x - r, y - r];
        let block_max = [x + r, y + r];
        let idx_min = self.in_block(block_min);
        let idx_max = self.in_block(block_max);
        for x in idx_min[X]..=idx_max[X] {
            for y in idx_min[Y]..=idx_max[Y] {
                let height = self.height([x, y]);
                if height >= z {
                    return true;
                }
            }
        }
        false
    }
    pub fn collide_line(&self, p1: [f64; 3], p2: [f64; 3]) -> bool {
        let fix_offs_rate = 0.001;

        // 如果起始点和目标点有碰撞就直接返回碰撞。
        if self.collide_point(p1) || self.collide_point(p2) {
            return true;
        }
        if p1[X] == p2[X] && p1[Y] == p2[Y] {
            return false;
        }

        let mut delta = 0.; // this is done
        loop {
            let delta_offs_x = next_step(p1[X], p2[X], delta, self.block_width, self.base[X]);
            let delta_offs_y = next_step(p1[Y], p2[Y], delta, self.block_width, self.base[Y]);
            let next_delta_offs = delta_offs_x.min(delta_offs_y);
            let delta_offs_fix =
                (delta_offs_x - delta_offs_y).abs().min(next_delta_offs) * fix_offs_rate;
            let delta_offs_fix = self.block_width() / p1.sub(p2).length() * fix_offs_rate;
            assert!(delta_offs_fix > 0., "delta_offs_fix = {}", delta_offs_fix);
            assert!(
                next_delta_offs > 0.,
                "next_delta_offs = {}",
                next_delta_offs
            );
            let delta_low = delta + next_delta_offs - delta_offs_fix;
            let delta_high = delta + next_delta_offs + delta_offs_fix;
            if delta_low < 1. && self.collide_point((p1, p2).lerp(delta_low)) {
                return true;
            }
            if delta_high < 1. && self.collide_point((p1, p2).lerp(delta_high)) {
                return true;
            }
            delta = delta_high;
            if delta >= 1. {
                break;
            }
        }
        false
    }
}

pub fn next_step(first: f64, second: f64, now: f64, width: f64, base: f64) -> f64 {
    if first == second {
        return 1.;
    }
    assert_ne!(first, second);
    let now_pos = [first, second].lerp(now);
    if first < second {
        let offs = now_pos - base;
        let remain = rempos(offs, width);
        assert!(remain >= 0.);
        assert!(remain <= width);
        let a = second - first;
        // dbg!(a);
        assert!(a > 0.);
        (width - remain) / a
    } else {
        let offs = now_pos - base;
        let remain = -rempos(offs, width);
        assert!(remain <= 0.);
        let a = second - first;
        assert!(a < 0.);
        remain / a
    }
    // let now =
}

#[test]
fn r() {
    use rand::random;

    let gen_rand = false;
    let test_pos = true;
    let test_neg = true;
    if gen_rand {
        dbg!(random::<f32>() * 200. - 100.);
        dbg!(random::<f32>() * 200. - 100.);
        dbg!(random::<f32>() * 200. - 100.);
        dbg!(random::<f32>());
        dbg!(random::<f32>() * 10.);
    }

    if test_pos {
        let left = -55.83328;
        let right = 43.80513;
        let base = -80.85533;
        let now_delta = 0.46742898;
        let block = 1.766355;

        let now_lerped = (left, right).lerp(now_delta);
        dbg!(now_lerped);
        dbg!(now_lerped - base);
        let next_offs = dbg!(((now_lerped - base) / block).ceil());
        let next = dbg!(base + block * next_offs);
        dbg!(next - now_lerped);
        let next_delta = (next - left) / (right - left);
        dbg!(next, (left, right).lerp(next_delta));
        dbg!(right - left);
        dbg!(next_delta - now_delta);
        assert!(
            next_step(left, right, now_delta, block, base) - (next_delta - now_delta)
                < 0.0000000001
        );
    }

    if test_neg {
        let left = 36.39627;
        let right = -84.05739;
        let base = -22.393417;
        let now_delta = 0.73866737;
        let block = 3.199309;

        let now_lerped = (left, right).lerp(now_delta);
        dbg!(now_lerped);
        dbg!(now_lerped - base);
        let next_offs = dbg!(((now_lerped - base) / block).floor());
        let next = dbg!(base + block * next_offs);
        assert!(next < now_lerped);
        dbg!(next - now_lerped);
        let next_delta = (next - left) / (right - left);
        assert!(next_delta > now_delta);
        dbg!(next, (left, right).lerp(next_delta));
        dbg!(right - left);
        dbg!(next_delta - now_delta);
        assert!(
            next_step(left, right, now_delta, block, base) - (next_delta - now_delta)
                < 0.0000000001
        );
    }
}
