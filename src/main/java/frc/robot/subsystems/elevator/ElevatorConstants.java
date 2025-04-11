// Copyright (c) 2025 FRC 10219
// https://github.com/team10219
// All rights reserved.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class ElevatorConstants {
  public enum Setpoint {
    kSoruce,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
  }

  public static final class ElevatorEncoderSetpoints {
    public static final LoggedTunableNumber kSource =
        new LoggedTunableNumber("Elevator/kSource", 0);
    public static final LoggedTunableNumber kLevel1 =
        new LoggedTunableNumber("Elevator/kLevel1", 9);
    public static final LoggedTunableNumber kLevel2 =
        new LoggedTunableNumber("Elevator/kLevel2", 18);
    public static final LoggedTunableNumber kLevel3 =
        new LoggedTunableNumber("Elevator/kLevel3", 29);
    public static final LoggedTunableNumber kLevel4 =
        new LoggedTunableNumber("Elevator/kLevel4", 45);

    // public static final int kSource = 0;
    // public static final int kLevel1 = 9;
    // public static final int kLevel2 = 18;
    // public static final int kLevel3 = 29;
    // public static final int kLevel4 = 45;
  }

  public static final class ElevatorValues {
    public static final int kLeaderCAN = 2;
    public static final int kFollowerCAN = 3;
  }

  public static final class SimulationConstants {
    // Elevator
    public static final double kPixelsPerMeter = 20;
    public static final double kElevatorGearRatio = 9.0;
    public static final double kCarriageMass = Units.lbsToKilograms(1.32);
    public static final double kElevatorDrumRadius = Units.inchesToMeters(1.76);
    public static final double kElevatorMinHeight = Units.inchesToMeters(39.5);
    public static final double kElevatorMaxHeight = Units.inchesToMeters(79);
  }
}
