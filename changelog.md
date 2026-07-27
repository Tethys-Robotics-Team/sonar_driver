# Changelog

## [1.0.2] - 2026-07-27

### Fixed
- Removed double publishing of sonar message
- Potential overflow issue that came from compariing message callback's size and uint_t

## [1.0.1] - 2025-10-30

### Fixed
- Removed print statements for debugging

## [1.0.0] - 2024-10-15

### Added
- Support for 3000 Series sonar from Bluepring Subsea
- Synched publication of Bearing table for each new image
- Synched publication of Uniform image for each new image
- Synched publication of Cartesian image for each new image
- Synched publication of sonar configuration for each new image

### Changed
- Changes to leverage C++17 features
- Implementation structure to reduce copying of images

## [0.1.0] - 2023-09-18

### Changed
- Sonar images and SonarConfiguration messages are now published synchronously with guaranteed identical timestamps.
