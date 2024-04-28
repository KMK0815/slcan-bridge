# `slcan-bridge`

Device to transfer messages on a canbus over USB serial as slcan format messages.

## Software

Once the device is plugged in, it should be at /dev/ttyACM0 or
similar. To check, use dmesg or similar.  To setup serial over
socketcan or slcan:

```
sudo slcand -S115200 -o -s5 ttyACM0 can0
sudo ip link set can0 up
ip link show can0
```

## Hardware

- [Nucleo-F103RB](https://www.st.com/en/evaluation-tools/nucleo-f103rb.html)

### Dev Board Hardware Pin assignments

The MCU Pin column is adjacent to headers it is connected to, ie
PB8 is on CN5-10 and CN10-3. Odd columns start at pin number '1' and even
columns start at pin number '2'.

#### CN6 Arduino and CN7 Extension

| MCU Pin |  CN7 Odd   | CN7 Even  | MCU Pin | CN6 |
|--------:|-----------:|----------:|--------:|----:|
|  PC10   |            |           | PC11    |     |
|  PC12   |            |           |  PD2    |     |
|   VDD   |            |           |  E5V    |     |
| BOOT0   |            |           |  GND    |     |
|    NC   |            |           |   NC    |  1  |
|    NC   |            |           | IOREF   |  2  |
|  PA13   |            |           | RESET   |  3  |
|  PA14   |            |           | +3V3    |  4  |
|  PA15   |            |           |  +5V    |  5  |
|   GND   |            |           |  GND    |  6  |
|   PB7   |            |           |  GND    |  7  |
|  PC13   |            |           |  VIN    |  8  |

#### CN5 Arduino and CN10 Extension

| CN5 | MCU Pin | CN10 Odd  | MCU Pin | CN10 Even |
|----:|--------:|----------:|--------:|----------:|
|     |   PC9   |           |  PC8    |           |
| 10  |   PB8   | CANRX     |  PC6    |           |
|  9  |   PB9   | CANTX     |  PC5    |           |
|  8  |  AVDD   |           |  U5V    |           |
|  7  |   GND   |           |  NC     |           |
|  6  |   PA5   |           |  PA12   |           |
|  5  |   PA6   |           |  PA11   |           |
|  4  |   PA7   |           |  PB12   |           |
|  3  |   PB6   |           |  PB11   |           |
|  2  |   PC7   | LED2      |  GND    |           |
|  1  |   PA9   | LED1      |  PB2    |           |


## Dependencies

#### 1. `flip-link`:

```console
$ cargo install flip-link
```

#### 2. `probe-rs`:

``` console
$ # make sure to install v0.2.0 or later
$ cargo install probe-rs --features cli
```

#### 7. Run!

You are now all set to `cargo-run` your first `defmt`-powered application!
There are some examples in the `src/bin` directory.

Start by `cargo run`-ning `my-app/src/bin/hello.rs`:

``` console
$ # `rb` is an alias for `run --bin`
$ cargo rb hello
    Finished dev [optimized + debuginfo] target(s) in 0.03s
flashing program ..
DONE
resetting device
0.000000 INFO Hello, world!
(..)

$ echo $?
0
```

If you're running out of memory (`flip-link` bails with an overflow error), you can decrease the size of the device memory buffer by setting the `DEFMT_RTT_BUFFER_SIZE` environment variable. The default value is 1024 bytes, and powers of two should be used for optimal performance:

``` console
$ DEFMT_RTT_BUFFER_SIZE=64 cargo rb hello
```

## Running tests

The template comes configured for running unit tests and integration tests on the target.

Unit tests reside in the library crate and can test private API; the initial set of unit tests are in `src/lib.rs`.
`cargo test --lib` will run those unit tests.

``` console
$ cargo test --lib
(1/1) running `it_works`...
└─ app::unit_tests::__defmt_test_entry @ src/lib.rs:33
all tests passed!
└─ app::unit_tests::__defmt_test_entry @ src/lib.rs:28
```

Integration tests reside in the `tests` directory; the initial set of integration tests are in `tests/integration.rs`.
`cargo test --test integration` will run those integration tests.
Note that the argument of the `--test` flag must match the name of the test file in the `tests` directory.

``` console
$ cargo test --test integration
(1/1) running `it_works`...
└─ integration::tests::__defmt_test_entry @ tests/integration.rs:13
all tests passed!
└─ integration::tests::__defmt_test_entry @ tests/integration.rs:8
```

Note that to add a new test file to the `tests` directory you also need to add a new `[[test]]` section to `Cargo.toml`.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
licensed as above, without any additional terms or conditions.

[Knurling]: https://knurling.ferrous-systems.com
[Ferrous Systems]: https://ferrous-systems.com/
[GitHub Sponsors]: https://github.com/sponsors/knurling-rs
