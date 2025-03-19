use std::{f64::consts::PI, fs, path::PathBuf};

use box_gen::{cacl::point::PointTrait, path_planning::Crane};
use clap::Parser;
use serde::Deserialize;
const DEG_TO_RAD: f64 = PI / 180.;
const RAD_TO_DEG: f64 = 180. / PI;

#[derive(Debug, Deserialize)]
struct Vars {
    row: f64,
    col: f64,
    laser: f64,
    target_heading: f64,
    autobodyx: f64,
    autobodyy: f64,
    autobodyz: f64,
    rotation: f64,
    amplitude: f64,
    rope: f64,
    臂长: f64,
    变幅相对回转中心偏移: f64,
    // 目标点局部坐标系: [f64; 3],
    就位点相对坐标X: f64,
    就位点相对坐标Y: f64,
    就位点相对坐标Z: f64,
    yaw_offs: f64,
}
#[derive(Debug, Parser)]
struct Args {
    input: PathBuf,
    output: PathBuf,
}

fn main() {
    // dbg!(DEG_TO_RAD * 5. * 24.);
    let args = Args::parse();
    // 读取json文件
    let vars = serde_json::from_str::<Vars>(&fs::read_to_string(args.input).unwrap()).unwrap();
    dbg!(&vars);
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    let 吊臂长度 = vars.臂长;
    let 变幅中心对回转中心偏移 = vars.变幅相对回转中心偏移;
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!

    let direction_vec = [vars.col, vars.row, 7.8212];

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!
    let position_vec = direction_vec.scale(vars.laser / direction_vec[2]);

    dbg!(position_vec);
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!
    let yaw_deg_gps = vars.target_heading;

    let yaw_deg = 270. - (yaw_deg_gps - vars.yaw_offs);
    let yaw_rad = yaw_deg * DEG_TO_RAD;
    dbg!((yaw_deg_gps - vars.yaw_offs));
    dbg!(yaw_rad * RAD_TO_DEG);

    let sin = yaw_rad.sin();
    let cos = yaw_rad.cos();
    dbg!((sin, cos));
    let [x, y, z] = position_vec;
    let [x, y] = [x * cos - y * sin, x * sin + y * cos];
    let position_vec = [x, y, z];
    dbg!(position_vec);
    // let position_vec = [0., 0., 0.];

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    let 吊车世界坐标系 = [vars.autobodyx, vars.autobodyy, vars.autobodyz];
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!
    let 定位点世界坐标系 = [0., 0., 0.];

    let 吊车局部坐标系 = 吊车世界坐标系.sub(定位点世界坐标系);
    // let 定位点局部坐标系 = [0., 0., 0.];
    let 起始点局部坐标系 = position_vec.scale(1.);
    dbg!(起始点局部坐标系);
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    let 目标点局部坐标系 = [
        vars.就位点相对坐标X,
        vars.就位点相对坐标Y,
        vars.就位点相对坐标Z,
    ];

    let crane = Crane::new(&(吊臂长度, 变幅中心对回转中心偏移, 吊车局部坐标系));
    let (t00, t10, l0) = dbg!(crane.position_to_pose(起始点局部坐标系));
    let (t01, t11, l1) = dbg!(crane.position_to_pose(目标点局部坐标系));
    let (dt0, dt1, dl) = dbg!(((t01 - t00) * RAD_TO_DEG, (t11 - t10) * RAD_TO_DEG, l1 - l0));

    let (st0, st1, sl) = (vars.rotation, vars.amplitude, vars.rope);
    let (dt0, dt1, dl) = (dt0, dt1, dl);
    let mut o = String::new();
    o += &format!("{:.4},{:.4},{:.4}\n", st0, st1, sl);
    o += &format!("{:.4},{:.4},{:.4}\n", st0 + dt0, st1, sl);
    o += &format!("{:.4},{:.4},{:.4}\n", st0 + dt0, st1 + dt1, sl);
    o += &format!("{:.4},{:.4},{:.4}\n", st0 + dt0, st1 + dt1, sl + dl);
    fs::write(args.output, o).unwrap();
}
