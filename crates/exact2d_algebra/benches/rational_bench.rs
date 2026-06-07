use criterion::{black_box, criterion_group, criterion_main, Criterion};
use exact2d_algebra::Rational;

fn bench_mul_1m(c: &mut Criterion) {
    let a = Rational::new(
        exact2d_integer::Integer::from(355i64),
        exact2d_integer::Integer::from(113i64),
    );
    let b = Rational::new(
        exact2d_integer::Integer::from(7i64),
        exact2d_integer::Integer::from(22i64),
    );

    c.bench_function("rational_mul_1M", |bencher| {
        bencher.iter(|| {
            let mut acc = Rational::one();
            for _ in 0..1_000_000 {
                // Keep the inputs stable to prevent numerator/denominator bit size
                // from growing exponentially to millions of digits.
                acc = black_box(a.clone()) * black_box(b.clone());
            }
            acc
        });
    });
}

criterion_group!(benches, bench_mul_1m);
criterion_main!(benches);
