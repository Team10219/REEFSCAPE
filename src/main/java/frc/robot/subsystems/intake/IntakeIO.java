// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.ControlType;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean leftConnected = false;
    public double leftPositionRads = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double leftTempCelsius = 0.0;
    public ControlType leftControlType = null;

    public boolean rightConnected = false;
    public double rightPositionRads = 0.0;
    public double rightVelocityRadPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
    public double rightTempCelsius = 0.0;
    public ControlType rightControlType = null;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void runSeperateVolts(double leftVolts, double rightVolts) {}

  default void stop() {}

  default void runVelocity(double velocity) {}

  default void runVelocityMAXMotion(double velocity) {}

  default void setPIDV(double kP, double kI, double kD, double vF) {}

  default void setBrakeMode(boolean enabled) {}
}
