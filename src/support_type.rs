use std::io::{self, Write};

use smallvec::SmallVec;

use crate::vec2d::Vec2d;

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
    block: f32,
}
impl Area {
    pub fn new(l: usize, base: [f32; 2], block: f32, zmin: f32, zmax: f32) -> Self {
        Self {
            data: Vec2d::new_filled(l, l, (SmallVec::new(), f32::NEG_INFINITY)),
            base,
            block,
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
            (dx / self.block).floor() as usize,
            (dy / self.block).floor() as usize,
        ]
    }
    pub fn pos(&self, pos: [usize; 2]) -> [f32; 2] {
        [
            (pos[0] as f32 * self.block + self.base[0]),
            (pos[1] as f32 * self.block + self.base[1]),
        ]
    }
    pub fn write_to_obj<W: Write>(&self, file: &mut W) -> io::Result<()> {
        for ((x, y), z) in self.data.iter().map(|(idx, (_, h))| (idx, *h)) {
            let [x, y] = self.pos([x, y]);
            write!(
                file,
                "v {} {} {}\n",
                (x + 0.5 * self.block),
                z,
                -(y + 0.5 * self.block)
            )?;
        }
        Ok(())
    }
}
impl Area {
    pub fn write_info<W: Write>(&self, file: &mut W) -> io::Result<()> {
        writeln!(file, "lowest {}", self.zmin)?;
        writeln!(file, "base xy {:?}", self.base)?;
        writeln!(file, "block_size {}", self.block)?;
        Ok(())
    }
    pub fn write_to_box_x<W: Write>(&self, file: &mut W) -> io::Result<()> {
        for ((x, y), _) in self.data.iter().map(|(idx, (_, h))| (idx, *h)) {
            let [x, _] = self.pos([x, y]);
            write!(file, "{} ", (x + 0.5 * self.block) - self.block * 0.5)?;
        }
        write!(file, "\n")?;
        Ok(())
    }
    pub fn write_to_box_y<W: Write>(&self, file: &mut W) -> io::Result<()> {
        for ((x, y), _) in self.data.iter().map(|(idx, (_, h))| (idx, *h)) {
            let [_, y] = self.pos([x, y]);
            write!(file, "{} ", -(y + 0.5 * self.block) - self.block * 0.5)?;
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
            write!(file, "{} ", self.block)?;
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
