#[macro_export]
macro_rules! time {
    ($st:ident) => {
        let $st = ::std::time::Instant::now();
    };
    ($st:ident, $info:literal) => {
        ::std::println!("{}: {:?}", $info, ::std::time::Instant::elapsed(&$st));
    };
}
