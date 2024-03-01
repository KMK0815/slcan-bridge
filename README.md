# `slcan-bridge`

Device to transfer messages on a canbus over USB serial as slcan format messages.

## Hardware

- [Nucleo-F103RB](https://www.st.com/en/evaluation-tools/nucleo-f303re.html)

### Dev Board Hardware Pin assignments

#### CN10 Connector

| MCU Pin | MCU Pin | CN10 Even  | CN10 Odd |
|--------:|--------:|-----------:|---------:|
|   PC9   |  PC8    |            |          |
|   PB8   |  PC6    | I2C1_SCL   |          |
|   PB9   |  PC5    | I2C1_SDA   |          |
|  AVDD   |  U5V    |            |          |
|   GND   |  NC     |            |          |
|   PA5   |  PA12   | SPI1_SCK   |          |
|   PA6   |  PA11   | SPI1_MISO  |          |
|   PA7   |  PB12   | SPI1_MOSI  |          |
|   PB6   |  PB11   | EPD_CS     |          |
|   PC7   |  GND    | D/C        |          |
|   PA9   |  PB2    | USART1_TX  |          |

#### CN9 Connector

| MCU Pin | MCU Pin | CN9 Even   | CN9 Odd  |
|--------:|--------:|-----------:|---------:|
|   PA8   |  PB1    |            |          |
|  PB10   | PB15    |            |          |
|   PB4   | PB14    | RST        |          |
|   PB5   | PB13    | BUSY       |          |
|   PB3   | AGND    | ENA        |          |
|  PA10   |  PC4    | USART1_RX  |          |
|   PA2   |  NC     | Set        |          |
|   PA3   |  NC     | Reset      |          |

## Removing Nucleo st-link pcb section
The portion of the dev board containing the st-link functionality can
be removed from the Nucleo. The remaining board must then be powered
through CN7 using Vin.

The board can still be programmed by connecting to CN7-7 [SWDIO] and
CN7-15 [SWCLK] from wires on CN4 on the st-link pcb.

### Jumper setting to use st-link externally, ie by using the jumper wires

- CN2 jumpers both OFF

### SWD connector CN4 on st-link pcb

| Logic Signal | Pin No |
|-------------:|-------:|
| VDD Target   |      1 |
| SWCLK        |      2 |
| GND          |      3 |
| SWDIO        |      4 |
| NRST         |      5 |
| SWO          |      6 |

## Jumper Configuration for Vin power supply for Nucleo

- JP5 pins 2 & 3 connected (Towards E5V label)
- JP1 OFF

The following power sequence procedure must be respected:
1. Connect the jumper between pin 2 and pin 3 of JP5.
2. Check that JP1 is removed.
3. Connect the external power source to VIN or E5V.
4. Power on the external power supply 7 V< VIN < 12 V to VIN, or 5 V for E5V.
5. Check that LD3 is turned ON.
6. Connect the PC to USB connector CN1.

If this order is not respected, the board may be supplied by VBUS first then by VIN or E5V,
and the following risks may be encountered:

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
