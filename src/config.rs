use serde::{Deserialize, Serialize};
use std::io::Write;
use std::{fs, path::PathBuf};

use crate::utils::StringErr;

#[derive(Debug, Serialize, Deserialize)]
pub struct AppConfig {
    pub rrt步长对block倍数: f64,
    pub rrt最大尝试采样次数: usize,
    pub rrt路径生成次数: usize,
    pub rrt终止点最大距离对block倍数: f64,

    pub 文件名称: PathBuf,
    pub 网格分割数量: usize,
    pub 上方偏移: f64,
    pub 下方偏移: f64,
    pub 偏移: f64,
    pub 包围盒扩大数量: usize,

    pub 起始点: [f64; 3],
    pub 目标点: [f64; 3],
    pub 吊臂长度: f64,
    pub 吊车回转中心水平坐标和变幅中心垂直坐标: [f64; 3],
    pub 吊车最大变幅角度: f64,
    pub 变幅中心对回转中心偏移: f64,
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            rrt步长对block倍数: 5.0,
            rrt最大尝试采样次数: 5000,
            rrt路径生成次数: 1024,
            rrt终止点最大距离对block倍数: 2.0,

            文件名称: r#"C:\Users\yaoyj\Desktop\2024-12-10\env.obj"#.into(),
            网格分割数量: 300,
            上方偏移: 20.,
            下方偏移: 20.,
            偏移: 0.02,
            包围盒扩大数量: 5,

            起始点: [7.011840, -2.013177, 0.397000],
            目标点: [7.107685, 0.592520, 0.928000],
            吊臂长度: 5.15744,
            吊车回转中心水平坐标和变幅中心垂直坐标: [
                2.493617, -1.038677, 1.021000,
            ],
            吊车最大变幅角度: 75.,
            变幅中心对回转中心偏移: -0.252292,
        }
    }
}

lazy_static::lazy_static!(
    pub static ref CONFIG:AppConfig = new_config();
);

fn new_config() -> AppConfig {
    let file = "config.json";
    let f: Result<AppConfig, String> = (|| {
        let f = fs::read_to_string(file).to_msg()?;
        dbg!(&f);
        let a = serde_json::from_str(&f).to_msg()?;
        Ok(a)
    })();
    match f {
        Ok(f) => f,
        Err(e) => {
            dbg!(e);
            let c = AppConfig::default();
            let s = serde_json::to_string_pretty(&c).unwrap();
            let mut f = fs::OpenOptions::new()
                .write(true)
                .truncate(true)
                .create(true)
                .open(file)
                .unwrap();
            write!(f, "{}", s).unwrap();

            c
        }
    }
}