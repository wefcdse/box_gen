use std::fs;

use stupid_utils::prelude::MutableInit;

fn main() {
    let senser = "./s.txt";
    let diff = "./d.txt";
    let out = "./o.csv";

    let (st0, st1, sl) = {
        let f = fs::read_to_string(senser).unwrap();
        let (t0, t1, l) = f
            .split(",")
            .mutable_cast(|f| (f.next().unwrap(), f.next().unwrap(), f.next().unwrap()));
        dbg!(t0, t1, l);
        let (t0, t1, l) = (t0.trim(), t1.trim(), l.trim());
        let (t0, t1, l) = (clean_up(t0), clean_up(t1), clean_up(l));
        dbg!(&t0, &t1, &l);
        let (t0, t1, l): (f64, f64, f64) =
            (t0.parse().unwrap(), t1.parse().unwrap(), l.parse().unwrap());
        (t0, t1, l)
    };
    let (dt0, dt1, dl) = {
        let f = fs::read_to_string(diff).unwrap();
        let (t0, t1, l) = f
            .split(",")
            .mutable_cast(|f| (f.next().unwrap(), f.next().unwrap(), f.next().unwrap()));
        dbg!(t0, t1, l);
        let (t0, t1, l) = (t0.trim(), t1.trim(), l.trim());
        let (t0, t1, l) = (clean_up(t0), clean_up(t1), clean_up(l));
        let (t0, t1, l): (f64, f64, f64) =
            (t0.parse().unwrap(), t1.parse().unwrap(), l.parse().unwrap());
        (t0, t1, l)
    };
    let mut o = String::new();
    o += &format!("{},{},{}\n", st0, st1, sl);
    o += &format!("{},{},{}\n", st0 + dt0, st1, sl);
    o += &format!("{},{},{}\n", st0 + dt0, st1 + dt1, sl);
    o += &format!("{},{},{}\n", st0 + dt0, st1 + dt1, sl + dl);
    fs::write(out, o).unwrap();
    // dbg!(s);
}

fn clean_up(s: &str) -> String {
    s.chars()
        .filter(|s| !s.is_whitespace() && *s != '\u{feff}')
        .collect()
}
