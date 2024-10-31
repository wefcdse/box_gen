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
    face: Vec2d<SmallVec<[usize; 4]>>,
    height: Vec2d<f32>,
    // a: SmallVec<[usize; 15]>,
    base: [f32; 2],
    block: f32,
}
impl Area {
    pub fn new(l: usize, base: [f32; 2], block: f32) -> Self {
        Self {
            face: Vec2d::new_filled(l, l, SmallVec::new()),
            height: Vec2d::new_filled(l, l, f32::NEG_INFINITY),
            base,
            block,
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
}
