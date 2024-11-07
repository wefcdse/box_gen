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

pub mod index_xyz {
    use std::ops::{Index, IndexMut};

    #[derive(Debug, Clone, Copy, Default, Hash, PartialEq, Eq, PartialOrd, Ord)]
    pub struct X;
    #[derive(Debug, Clone, Copy, Default, Hash, PartialEq, Eq, PartialOrd, Ord)]
    pub struct Y;
    #[derive(Debug, Clone, Copy, Default, Hash, PartialEq, Eq, PartialOrd, Ord)]
    pub struct Z;
    // impl<const L: usize, T> Index<X> for [T; L] {
    //     type Output = T;

    //     fn index(&self, _: X) -> &Self::Output {
    //         &self[0]
    //     }
    // }
    // impl<const L: usize, T> IndexMut<X> for [T; L] {
    //     fn index_mut(&mut self, _: X) -> &mut Self::Output {
    //         &mut self[0]
    //     }
    // }
    macro_rules! implthis {
        ($a:tt, $b:literal) => {
            impl<const L: usize, T> Index<$a> for [T; L] {
                type Output = T;

                fn index(&self, _: $a) -> &Self::Output {
                    &self[$b]
                }
            }
            impl<const L: usize, T> IndexMut<$a> for [T; L] {
                fn index_mut(&mut self, _: $a) -> &mut Self::Output {
                    &mut self[$b]
                }
            }
        };
    }
    implthis!(X, 0);
    implthis!(Y, 1);
    implthis!(Z, 2);
    #[test]
    fn t() {
        let mut a = [318, 15, 5];
        assert_eq!(a[0], a[X]);
        assert_eq!(a[1], a[Y]);
        assert_eq!(a[2], a[Z]);
        assert!(*(&mut a[0]) == *(&mut a[X]));
        assert!(*(&mut a[1]) == *(&mut a[Y]));
        assert!(*(&mut a[2]) == *(&mut a[Z]));

        let mut a = [318, 15];
        assert_eq!(a[0], a[X]);
        assert_eq!(a[1], a[Y]);
        assert!(*(&mut a[0]) == *(&mut a[X]));
        assert!(*(&mut a[1]) == *(&mut a[Y]));
    }
}
