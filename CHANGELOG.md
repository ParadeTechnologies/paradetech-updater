# Changelog

All notable changes to this project will be documented in this file, in reverse chronological order by release.

## [0.7.1] - 2025-09-09

### Changed
- Add calibration support for ptupdater
- Synchronize latest PtLib
- Update version of ptupdater to 0.7.1

### Fixed
- Fixed ptupdater crash issue when no enough permission to open hidraw node

## [0.7.0] - 2025-06-20

### Changed
- Remove PIP2 support
- Remove PTU file support
- Synchronize latest PtLib
- Update version of ptupdater to 0.7.0

## [0.6.6] - 2025-06-16

### Fixed
- Fixed exit bl failure after firmware update when primary image is broken

### Changed
- Synchronize latest PtLib
- Update version of ptupdater to 0.6.6

## [0.6.5] - 2025-04-21

### Added
- Add enum for FW category
- Add support for PIP2 and PIP3 FILE_IOCTL file_crc (ioctl code: 0x04)
- Add catagory ID for bootloader firmware

### Fixed
- Fixed typo mistake in CHANGELOG.md

### Changed
- Revise normal workaround not to use pthread_kill() to detect whether a thread terminates
- Clean codes in PtLib/dut_utils: remove unused variable version_rsp
- Update version of ptupdater to 0.6.5

## [0.6.4] - 2024-05-23

### Fixed
- Update pttupdater to not requesting hidraw node if only "check_target"
- Update helper print

### Changed
- Update version of ptupdater to 0.6.4

## [0.6.3] - 2024-04-10

### Added
- Add file CHANGELOG.md

### Fixed
- Revert "Fix minijail stuck issue if use pthread_exit() to exit a thread",
 it turns out to be policy issue
- Fix ptupdater not to send a command in function teardown_pip3_api() which
 results an error if primary image is broken

### Changed
- Update version of ptupdater to 0.6.3

## [0.6.2] - 2024-03-26

### Fixed
- Fix Makefile to use CFLAGS, CPPFLAGS, LDFLAGS correctly
- Remove build option -s,  -static for Chromeos
- Fix dead loop in function start_hidraw_report_reader()
- Fix minijail stuck issue if use pthread_exit() to exit a thread
- Fix 2 compiler warnings:
  - Remove const declaration for flash_files_to_erase_id_list[]
  -  Add initial value for max_output_len in function  _get_max_output_len()

### Changed
- Update version of ptupdater to 0.6.2

## [0.6.1] - 2024-03-21

### Fixed
- Fix default rc value in function process_fw_file()

### Changed
- Update version of ptupdater to 0.6.1

## [0.6.0] - 2024-03-21

### Changed
- Firmware file will be stored in binary file instead of  ptu file
-- Remove the ptu_parser.c, ptu_parser.h from makefile
-- Add new function process_fw_file() to deal with binary firmware

## [0.5.1] - 2024-02-16

### Added
- Minor performance optimizations from first public Beta release v0.5.0,
namely the `--check-active` CLI argument now takes an average of about 32
milliseconds, whereas previously it previously took over 150 milliseconds.

## [0.5.0] - 2024-02-03

## Initial release
First public Beta release. Tool for updating the firmware on Parade
Technologies Touch devices on a Linux host (e.g., Chrome OS). See
README.md for more information.
(Note that v0.1.0 to v0.4.0 were internal-only releases.)

[0.7.1]: https://github.com/ParadeTechnologies/paradetech-updater/compare/v0.7.0...v0.7.1
[0.7.0]: https://github.com/ParadeTechnologies/paradetech-updater/compare/v0.6.6...v0.7.0
[0.6.6]: https://github.com/ParadeTechnologies/paradetech-updater/compare/v0.6.5...v0.6.6
[0.6.5]: https://github.com/ParadeTechnologies/paradetech-updater/compare/v0.6.4...v0.6.5
[0.6.4]: https://github.com/ParadeTechnologies/paradetech-updater/compare/v0.6.3...v0.6.4
[0.6.3]: https://github.com/ParadeTechnologies/paradetech-updater/compare/v0.6.2...v0.6.3
[0.6.2]: https://github.com/ParadeTechnologies/paradetech-updater/compare/v0.6.1...v0.6.2
[0.6.1]: https://github.com/ParadeTechnologies/paradetech-updater/compare/v0.6.0...v0.6.1
[0.6.0]: https://github.com/ParadeTechnologies/paradetech-updater/compare/v0.5.1...v0.6.0
[0.5.1]: https://github.com/ParadeTechnologies/paradetech-updater/compare/v0.5.0...v0.5.1
[0.5.0]: https://github.com/ParadeTechnologies/paradetech-updater/commits/v0.5.0
