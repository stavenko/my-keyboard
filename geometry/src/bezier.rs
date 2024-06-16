pub fn fact(n: usize) -> f32 {
    if n == 0 {
        1.
    } else {
        (1..=n).product::<usize>() as f32
    }
}

pub fn bernstein<const D: usize>(i: usize, t: f32) -> f32 {
    let d = D - 1;
    let ot = 1. - t;
    let ni = fact(d) / (fact(i) * fact(d - i));
    let di = d - i;
    ni * t.powi(i as i32) * ot.powi(di as i32)
}
