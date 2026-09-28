# BMP390

> **Local fork** of [`asasine/bmp390`](https://github.com/asasine/bmp390) 0.4.3, patched
> over crates.io in the workspace `Cargo.toml` (`[patch.crates-io]`). Changes for Elle:
> accepts the BMP384 chip ID (`0x50`, same register map) as well as `0x60`; `uom` 0.38;
> no `embassy-time` dependency (it clashed with the workspace's git embassy). The
> firmware uses the **asynchronous** driver on interrupt-driven I2C0, through an
> `embassy-embedded-hal` async `I2cDevice` shared with the magnetometer
> (`elle_hardware::imu::i2c_sensors`). The upstream text below still applies otherwise.


The BMP390 is a digital sensor with pressure and temperature measurement based on proven sensing principles. The sensor is more accurate than its predecessor BMP380, covering a wider measurement range. It offers new interrupt functionality, lower power, and a FIFO functionality. The integrated 512 byte FIFO buffer supports low power applications and prevents data loss in non-real-time systems.

[`Bmp390`](https://docs.rs/bmp390/latest/bmp390/struct.Bmp390.html) is a driver for the BMP390 sensor. It provides methods to read the temperature and pressure from the sensor over [I2C](https://en.wikipedia.org/wiki/I%C2%B2C). It is built on top of the [`embedded_hal_async::i2c`](https://docs.rs/embedded-hal-async/latest/embedded_hal_async/i2c/index.html) traits to be compatible with a wide range of embedded platforms. Measurements utilize the [`uom`](https://docs.rs/uom/latest/uom/) crate to provide automatic, type-safe, and zero-cost units of measurement for [`Measurement`](https://docs.rs/bmp390/latest/bmp390/struct.Measurement.html).

Synchronous and asynchronous interfaces are available. The synchronous interface is built on top of the [`embedded-hal`](https://docs.rs/embedded-hal/latest/embedded_hal/) traits, while the asynchronous interface is built on top of the [`embedded-hal-async`](https://docs.rs/embedded-hal-async/latest/embedded_hal_async/) traits. The default features include the *asynchronous* interface, but the synchronous [`sync::Bmp390`](https://docs.rs/bmp390/latest/bmp390/sync/struct.Bmp390.html) one can be enabled with the `sync` feature.

## Datasheet
The [BMP390 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf) contains detailed information about the sensor's features, electrical characteristics, and registers. This package implements the functionality described in the datasheet and references the relevant sections in the documentation.
