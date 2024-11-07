use std::io::{self, Write};

use smallvec::SmallVec;

use crate::{
    cacl::{lerp::Lerp, rempos::rempos},
    disable,
    utils::index_xyz::*,
    vec2d::Vec2d,
};

// 0####1@@@@
// #####@@@@@
// #####@@@@@
// 1$$$$
// $$$$$
// $$$$$
// 2
pub struct Area {
    pub data: Vec2d<(SmallVec<[usize; 4]>, f64)>,
    // pub height: Vec2d<f64>,
    // a: SmallVec<[usize; 15]>,
    base: [f64; 2],
    zmin: f64,
    #[allow(dead_code)]
    zmax: f64,
    block_width: f64,
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
        assert!(dx >= 0.);
        assert!(dy >= 0.);
        [
            (dx / self.block_width).floor() as usize,
            (dy / self.block_width).floor() as usize,
        ]
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

        let mut delta = 0.; // this is done
        loop {
            let delta_offs_x = next_step(p1[X], p2[X], delta, self.block_width, self.base[X]);
            let delta_offs_y = next_step(p1[Y], p2[Y], delta, self.block_width, self.base[Y]);
            let next_delta_offs = delta_offs_x.min(delta_offs_y);
            let delta_offs_fix =
                (delta_offs_x - delta_offs_y).abs().min(next_delta_offs) * fix_offs_rate;
            assert!(delta_offs_fix > 0.);
            assert!(next_delta_offs > 0.);

            if self.collide_point((p1, p2).lerp(delta + next_delta_offs - delta_offs_fix))
                || self.collide_point((p1, p2).lerp(delta + next_delta_offs + delta_offs_fix))
            {
                return true;
            }
            delta = delta + next_delta_offs + delta_offs_fix
        }
        false
    }
}

pub fn next_step(first: f64, second: f64, now: f64, width: f64, base: f64) -> f64 {
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
