# Code TODO List

- [x] Make everything static. No singletons.
- [x] Separate MotorInfo into MotorBuilder and EncoderBuilder
- [ ] Talking of naming, use better naming.
  - [x] MotorInfo -> MotorBuilder
  - [x] PIDControllerConstants -> PIDControllerBuilder
  - [ ] ShuffleControl -> Something not cringe
  - [ ] UpperArm and ForeArm -> SegOne and SegTwo
- [x] Move SimConstants into Constants
- [x] Split Constants into multiple files for easier navigation
- [x] Move constants in the Variables file (drive settings) into the Constants
- [ ] Make LEDControl a subsystem, for gods sake
- [ ] Custom ShuffleBoard component for visualizing arm (2D plotter,
      desmos-style)
