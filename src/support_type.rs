use std::io::{self, Write};

use smallvec::SmallVec;

use crate::{disable, utils::lerp::Lerp, vec2d::Vec2d};

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
        let dx = pos[0] - self.base[0];
        let dy = pos[1] - self.base[1];
        assert!(dx >= 0.);
        assert!(dy >= 0.);
        [
            (dx / self.block_width).floor() as usize,
            (dy / self.block_width).floor() as usize,
        ]
    }
    pub fn pos(&self, block: [usize; 2]) -> [f64; 2] {
        [
            (block[0] as f64 * self.block_width + self.base[0]),
            (block[1] as f64 * self.block_width + self.base[1]),
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
#[test]
fn write() {
    let mut f = std::fs::OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .open("exp.obj")
        .unwrap();
    let mut a1 = Area::new(5, [0., 0.], 2., 0., 0.);
    a1.data.iter_mut().for_each(|(_, (_, a))| *a = 4.5);
    a1.write_to_obj(&mut f).unwrap();
}

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
        for x in idx_min[0]..=idx_max[0] {
            for y in idx_min[1]..=idx_max[1] {
                let height = self.height([x, y]);
                if height >= z {
                    return true;
                }
            }
        }
        false
    }
    pub fn collide_line(&self, p1: [f64; 3], p2: [f64; 3]) -> bool {
        // 如果起始点和目标点有碰撞就直接返回碰撞。
        if self.collide_point(p1) || self.collide_point(p2) {
            return true;
        }

        let (plow, phigh) = {
            if p1[2] > p2[2] {
                (p2, p1)
            } else {
                (p1, p2)
            }
        };
        assert!(plow[2] <= phigh[2]);
        let step = {
            let s = f64::max((p1[0] - p2[0]).abs(), (p1[1] - p2[1]).abs());
            // 如果两个点差别不到1block
            if s < self.block_width {
                return false;
            }
        };

        disable!(p1, p2);

        todo!()
    }
}

pub fn next_step(first: f64, second: f64, now: f64, width: f64, base: f64) {
    let now_pos = [first, second].lerp(now);
    // let now =
}
