use std::io::{self, Write};

use smallvec::SmallVec;

use crate::{disable, vec2d::Vec2d};

// 0####1@@@@
// #####@@@@@
// #####@@@@@
// 1$$$$
// $$$$$
// $$$$$
// 2
pub struct Area {
    pub data: Vec2d<(SmallVec<[usize; 4]>, f32)>,
    // pub height: Vec2d<f32>,
    // a: SmallVec<[usize; 15]>,
    base: [f32; 2],
    zmin: f32,
    #[allow(dead_code)]
    zmax: f32,
    block_width: f32,
}
impl Area {
    pub fn new(l: usize, base: [f32; 2], block_width: f32, zmin: f32, zmax: f32) -> Self {
        Self {
            data: Vec2d::new_filled(l, l, (SmallVec::new(), f32::NEG_INFINITY)),
            base,
            block_width,
            zmin,
            zmax,
        }
    }
    pub fn in_block(&self, pos: [f32; 2]) -> [usize; 2] {
        let dx = pos[0] - self.base[0];
        let dy = pos[1] - self.base[1];
        assert!(dx >= 0.);
        assert!(dy >= 0.);
        [
            (dx / self.block_width).floor() as usize,
            (dy / self.block_width).floor() as usize,
        ]
    }
    pub fn pos(&self, block: [usize; 2]) -> [f32; 2] {
        [
            (block[0] as f32 * self.block_width + self.base[0]),
            (block[1] as f32 * self.block_width + self.base[1]),
        ]
    }
    pub fn height(&self, block: [usize; 2]) -> f32 {
        self.data[block].1
    }
    pub fn write_to_obj<W: Write>(&self, file: &mut W) -> io::Result<()> {
        for ((x, y), z) in self.data.iter().map(|(idx, (_, h))| (idx, *h)) {
            let [x, y] = self.pos([x, y]);
            write!(
                file,
                "v {} {} {}\n",
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
        write!(file, "\n")?;
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
        write!(file, "\n")?;
        Ok(())
    }
    pub fn write_to_box_z<W: Write>(&self, file: &mut W) -> io::Result<()> {
        for (_, _z) in self.data.iter().map(|(idx, (_, h))| (idx, *h)) {
            write!(file, "{} ", self.zmin)?;
        }
        write!(file, "\n")?;
        Ok(())
    }
    pub fn write_to_box_width<W: Write>(&self, file: &mut W) -> io::Result<()> {
        for _ in self.data.iter() {
            write!(file, "{} ", self.block_width)?;
        }
        write!(file, "\n")?;
        Ok(())
    }
    pub fn write_to_box_height<W: Write>(&self, file: &mut W) -> io::Result<()> {
        for (_, z) in self.data.iter().map(|(idx, (_, h))| (idx, *h)) {
            write!(file, "{} ", (z - self.zmin))?;
        }
        write!(file, "\n")?;
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
impl Area {
    pub fn collide_point(&self, pos: [f32; 3]) -> bool {
        let [x, y, z] = pos;
        disable!(pos);
        let block = self.in_block([x, y]);
        let height = self.height(block);
        height >= z
    }
    pub fn collide_area(&self, pos: [f32; 2], height: f32, r: f32) -> bool {
        let ([x, y], z) = (pos, height);
        disable!(pos, height);
        let block_min = [x - r, y - r];
        let block_max = [x + r, y + r];
        let idx_min = self.in_block(block_min);
        let idx_max = self.in_block(block_max);
        for x in idx_min[0]..idx_max[0] {
            for y in idx_min[1]..idx_max[1] {
                let height = self.height([x, y]);
                if height >= z {
                    return true;
                }
            }
        }
        false
    }
    fn collide_line(p1: [f32; 3], pos: [f32; 3]) {}
}
