- Integrated controller.
- %---------------------------Control mode select---------------------------%
  % 0: Not use      1: Current(PI)   2: Current(MPC)  3: Current(DOB)       %
  % 0: Not use      1: Velocity(PI)  2: Velocity(MPC) 3: Velocity(H-inf)    %
  % 0: Not use      1: Position(PI)                                         %
  %-------------------------------------------------------------------------%
- Velocity(MPC)&Velocity(H-inf) -> Both controllers control the velocity and current at the same time.
- Velocity(H-inf) controller is incomplete.