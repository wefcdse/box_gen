#[macro_export]
macro_rules! time {
    ($st:ident) => {
        let $st = ::std::time::Instant::now();
    };
    ($st:ident, $info:literal) => {
        ::std::println!("{}: {:?}", $info, ::std::time::Instant::elapsed(&$st));
    };
}
#[macro_export]
macro_rules! disable {
    ($($a:ident),*) => {
        $(
        #[allow(unused_variables)]
        let $a = {
            struct Disabled;
            Disabled
        };
        {
            $a;
        }
        )*
    };
}

pub mod lerp {
    pub trait Lerp: Copy {
        type Inner;
        type Float;
        fn lerp(self, delta: Self::Float) -> Self::Inner;
    }
    impl<T: ExtractInner2<f64>> Lerp for T {
        type Inner = f64;

        type Float = f64;

        fn lerp(self, delta: Self::Float) -> Self::Inner {
            let (a, b) = self.extract();
            a * (1. - delta) + b * delta
        }
    }
    trait ExtractInner2<Inner>: Copy {
        fn extract(self) -> (Inner, Inner);
    }
    impl<T: Into<f64> + Copy> ExtractInner2<f64> for (T, T) {
        fn extract(self) -> (f64, f64) {
            let (a, b) = self;
            (a.into(), b.into())
        }
    }
    impl<T: Into<f64> + Copy> ExtractInner2<f64> for [T; 2] {
        fn extract(self) -> (f64, f64) {
            let [a, b] = self;
            (a.into(), b.into())
        }
    }

    #[test]
    fn l() {
        assert_eq!((1i16, 0).lerp(0.5), 0.5);
        assert!(((1.3, -2.5).lerp(0.45) - -0.41) < 0.000001);
    }
}
