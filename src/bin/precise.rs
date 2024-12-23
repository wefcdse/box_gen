use std::{f64::consts::PI, fs};

use box_gen::{
    cacl::{jwd经纬度到xy, point::PointTrait},
    path_planning::{AsMove, Crane},
    utils::local_wrap::WrapSelf,
};

fn main() {
    let 吊臂长度 = 5.38626;
    let 变幅中心对回转中心偏移 = -0.262292;

    // let direction_vec = [2.3094311117840, 0.5238344, 7.8212];
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!
    let direction_vec = [1.7576, -2.8416899044, 7.8212];
    let direction_vec = [1.74822, 0.858877, 7.8212];
    let direction_vec = [2.0088712, 0.05344, 7.8212];
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!
    let position_vec = direction_vec.scale(88. / direction_vec[2]);

    dbg!(position_vec);
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!
    let yaw_deg_gps = 274.889;

    let yaw_deg = 270. - yaw_deg_gps;
    let deg_to_rad = PI / 180.;
    let yaw_rad = yaw_deg * deg_to_rad;

    let sin = yaw_rad.sin();
    let cos = yaw_rad.cos();
    let [x, y, z] = position_vec;
    let [x, y, z] = [y, x, z];
    // let [x, y] = [x * cos - y * sin, x * sin + y * cos];
    let position_vec = [x, y, z];
    dbg!(position_vec);
    // real = 0,-23

    // let 吊车经纬度高度 = [121.509592444, 38.880311618, 82.283];
    // let 定位坐标原点经纬度高度 = [121.509602248, 38.880344189, 81.979];
    // let dbg目标点经纬度高度 = [121.509611261, 38.880348400, 81.941];

    // let 吊车世界坐标系 = jwd经纬度到xy(吊车经纬度高度[0], 吊车经纬度高度[1])
    //     .wrap()
    //     .extend_one(吊车经纬度高度[2]);
    // let 定位点世界坐标系 = jwd经纬度到xy(定位坐标原点经纬度高度[0], 定位坐标原点经纬度高度[1])
    //     .wrap()
    //     .extend_one(定位坐标原点经纬度高度[2]);
    let 吊车世界坐标系 = [-1.213607, 3.269370, 0.97];
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!
    let 定位点世界坐标系 = [-1.055474437889643, -1.3575758170336485, 0.6529999999999987];
    let 定位点世界坐标系 = [-0.9611894839326851, -1.4975408306345344, 0.6529999999999987];
    // let dbg目标点世界坐标系 = jwd经纬度到xy(dbg目标点经纬度高度[0], dbg目标点经纬度高度[1])
    //     .wrap()
    //     .extend_one(dbg目标点经纬度高度[2]);

    let 吊车局部坐标系 = 吊车世界坐标系.sub(定位点世界坐标系);
    // let 定位点局部坐标系 = [0., 0., 0.];
    let 起始点局部坐标系 = position_vec.scale(0.01);
    dbg!(起始点局部坐标系);
    // let 目标点局部坐标系 = dbg目标点世界坐标系.sub(定位点世界坐标系);
    let 目标点局部坐标系 = [0., 0., 0.];
    // dbg!(吊车局部坐标系);
    // let 吊车局部坐标系 = [0., -4., 0.];
    // dbg!(吊车局部坐标系.length());
    // dbg!(目标点局部坐标系);
    let rad_to_deg = 180. / PI;
    let crane = Crane::new(&(吊臂长度, 变幅中心对回转中心偏移, 吊车局部坐标系));
    let (t00, t10, l0) = dbg!(crane.position_to_pose(起始点局部坐标系));
    let (t01, t11, l1) = dbg!(crane.position_to_pose(目标点局部坐标系));
    let (dt0, dt1, dl) = dbg!(((t01 - t00) * rad_to_deg, (t11 - t10) * rad_to_deg, l1 - l0));
    // dbg!(
    //     position[0].length(),
    //     position[1].length(),
    //     position[2].length()
    // );
    // let position: [[f64; 3]; 3] = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
    let normals: [[f64; 3]; 3] = crane.normals(起始点局部坐标系);
    let proj_to_local = nalgebra::Matrix3::from(normals);
    let local_to_proj = proj_to_local.try_inverse().unwrap();
    let path_direction = nalgebra::Vector3::from(目标点局部坐标系.sub(起始点局部坐标系));
    // let [t1, t2, l, ..] = *dbg!(local_to_proj * path_direction).as_slice();
    let nalgebra::coordinates::XYZ { x: t1, y: t2, z: l } = *dbg!(local_to_proj * path_direction);
    let (st1, st2, sl) = crane.step_at(起始点局部坐标系);
    let rad2deg = 180. / PI;
    let (st1, st2, sl) = (st1 * t1 * rad2deg, st2 * t2 * rad2deg, sl * l);
    dbg!(st1, st2, sl);
    fs::write("./temp/d.csv", format!("{},{},{}", dt0, dt1, dl)).unwrap();
    // dbg!(base.transpose().determinant());
    // println!("{:.5}", base);
    // dbg!(base.is_invertible());
    // dbg!(position);
}
