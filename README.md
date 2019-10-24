# geo-raycasting

A simple implementation of Ray Casting algorithm for geo crate, inspired by the code found on https://rosettacode.org/wiki/Ray-casting_algorithm

Example:
```rust
use geo_raycasting::RayCasting;

use geo_types::LineString;

fn main() {
    let poly_square: LineString<f64> = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)].into();
    assert!(poly_square.within(&(5.0, 5.0).into()));
}
```
