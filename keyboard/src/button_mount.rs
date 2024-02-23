#[derive(Clone, Debug)]
struct ButtonMount {
    width: Dec,
    height: Dec,
    lock_height: Dec,
    padding: Dec,
}

impl ButtonMount {
    fn chok() -> Self {
        Self {
            width: dec!(13.8).into(),
            height: dec!(13.8).into(),
            lock_height: dec!(1.2).into(),
            padding: dec!(0.7).into(),
        }
    }
}
