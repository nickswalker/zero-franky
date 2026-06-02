# Changelog

## [0.1.2] - Unreleased

### Added

- Wrap cartesian impedance damping parameters

### Fixed

- Remove a broken nullspace task parameter

## [0.1.1] - 2026-06-01

### Added

- Wrap provisional friction feedforward and nullspace task API

### Changed

- Relax Python version support to >=3.9 to match Franky
- Allow empty tracker session policy for basic impedance loops where you just use `set_reference` on the client
- Added context-manager cleanup for tracker session proxies.

### Fixed

- Sync motions work instead of generating a network timeout

## [0.1.0] - 2026-05-23

### Added

- Initial `zero-franky` package for controlling `franky` through a ZeroMQ client/server bridge.
- Msgpack protocol support for common robot commands, motion payloads, telemetry, and tracker sessions.
- Joint and Cartesian impedance tracker session support, including import-based and `cloudpickle` policy transports.

[0.1.1]: https://github.com/nickswalker/zero-franky/releases/tag/v0.1.1
[0.1.0]: https://github.com/nickswalker/zero-franky/releases/tag/v0.1.0
