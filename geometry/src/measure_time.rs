pub(crate) fn print_time(label: &str, from: std::time::Instant) -> std::time::Instant {
    let cur = std::time::Instant::now();
    let dur = cur - from;
    println!("{label}:  {}", dur.as_secs_f32());
    cur
}
