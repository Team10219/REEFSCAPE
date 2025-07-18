// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public String elevatorControlType = null;
    // public boolean atSetpoint = false;

    public boolean leaderConnected = false;
    public double leaderPositionRads = 0.0;
    public double leaderVelocityRadsPerSec = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderCurrentAmps = 0.0;
    public double leaderTempCelsius = 0.0;

    public boolean followerConnected = false;
    public double followerPositionRads = 0.0;
    public double followerVelocityRadsPerSec = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerCurrentAmps = 0.0;
    public double followerTempCelsius = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void setPosition(double position) {}

  default void setBrakeMode(boolean enabled) {}
}
