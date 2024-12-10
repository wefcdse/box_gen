use core::ffi;

use support_type::Area;

pub mod areagen;
pub mod cacl;
pub mod config;
pub mod path_planning;
pub mod support_type;
pub mod utils;
pub mod vec2d;

/// # Safety
/// file_name must be a valid c str
#[no_mangle]
pub unsafe extern "C" fn generate_from_obj(
    file_name: *const ffi::c_char,
    split: usize,
    offs_high: f64,
    offs_low: f64,
    offs: f64,
) -> Box<Area> {
    let file_name = ffi::CStr::from_ptr(file_name).to_str().unwrap();
    let area = Area::gen_from_obj_file(file_name, split, offs_high, offs_low, offs);
    Box::new(area)
}

#[no_mangle]
pub extern "C" fn collide_point(area: &Area, x: f64, y: f64, z: f64) -> bool {
    area.collide_point([x, y, z])
}

#[no_mangle]
pub extern "C" fn collide_line(
    area: &Area,
    x1: f64,
    y1: f64,
    z1: f64,
    x2: f64,
    y2: f64,
    z2: f64,
) -> bool {
    area.collide_line([x1, y1, z1], [x2, y2, z2])
}

#[no_mangle]
pub extern "C" fn debug_area(area: &Area) {
    dbg!(area);
}
#[no_mangle]
pub extern "C" fn box_gen_valid() -> bool {
    println!("valid!!!");
    true
}
#[repr(C)]
pub struct AreaBoundingBox {
    xmin: f64,
    ymin: f64,
    zmin: f64,
    xmax: f64,
    ymax: f64,
    zmax: f64,
}

#[no_mangle]
pub extern "C" fn get_bounding_box(area: &Area) -> AreaBoundingBox {
    let [xmin, ymin, zmin] = area.min();
    let [xmax, ymax, zmax] = area.max();
    AreaBoundingBox {
        xmin,
        ymin,
        zmin,
        xmax,
        ymax,
        zmax,
    }
}

#[no_mangle]
pub extern "C" fn free_area(area: Box<Area>) {
    drop(area);
}
