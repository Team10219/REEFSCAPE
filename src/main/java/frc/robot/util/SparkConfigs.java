// Copyright (c) 2025 FRC 10219
// https://github.com/team10219
// All rights reserved.

package frc.robot.util;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public class SparkConfigs {
  public static final class ElevatorConfigs {
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    public static final boolean elevatorBrakeModeEnabled = true;
    public static final boolean elevatorInverted = false;

    static {
      elevatorConfig
          .idleMode(
              elevatorBrakeModeEnabled
                  ? SparkMaxConfig.IdleMode.kBrake
                  : SparkMaxConfig.IdleMode.kCoast)
          .inverted(elevatorInverted);
      elevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(0.2)
          .d(0.08)
          .outputRange(-1, 1)
          .maxMotion
          .maxVelocity(4000)
          .maxAcceleration(6000)
          .allowedClosedLoopError(0.5);
      elevatorConfig.smartCurrentLimit(80).voltageCompensation(12);
    }
  }

  public static class IntakeConfigs {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    public static final boolean intakeBrakeModeEnabled = true;
    public static final boolean intakeInverted = true;

    static {
      intakeConfig
          .idleMode(
              intakeBrakeModeEnabled
                  ? SparkMaxConfig.IdleMode.kBrake
                  : SparkMaxConfig.IdleMode.kCoast)
          .inverted(intakeInverted);
    }
  }
}
