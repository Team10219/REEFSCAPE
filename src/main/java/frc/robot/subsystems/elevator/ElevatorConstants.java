// Copyright (c) 2025 FRC 10219
// https://github.com/team10219
// All rights reserved.

package frc.robot.subsystems.elevator;

import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class ElevatorConstants {
  public enum Setpoint {
    Soruce,
    Level1,
    Level2,
    Level3,
    Level4;
  }

  public static final class ElevatorEncoderSetpoints {
    public static final LoggedTunableNumber Source = new LoggedTunableNumber("Elevator/Source", 0);
    public static final LoggedTunableNumber Level1 = new LoggedTunableNumber("Elevator/Level1", 9);
    public static final LoggedTunableNumber Level2 = new LoggedTunableNumber("Elevator/Level2", 18);
    public static final LoggedTunableNumber Level3 = new LoggedTunableNumber("Elevator/Level3", 29);
    public static final LoggedTunableNumber Level4 = new LoggedTunableNumber("Elevator/Level4", 45);

    // public static final int Source = 0;
    // public static final int Level1 = 9;
    // public static final int Level2 = 18;
    // public static final int Level3 = 29;
    // public static final int Level4 = 45;
  }

  public static final class ElevatorCAN {
    public static final int leaderCAN = 2;
    public static final int followerCAN = 3;
  }
}
