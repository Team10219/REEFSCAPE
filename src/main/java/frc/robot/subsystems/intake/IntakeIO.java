// Copyright (c) 2025 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean leftConnected = false;
    public double leftVelocityRPM = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double leftTempCelsius = 0.0;
    public String leftControlType = null;

    public boolean rightConnected = false;
    public double rightAngularVelocityDPS = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
    public double rightTempCelsius = 0.0;
    public String rightControlType = null;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void runVelocity(double velocity) {}

  default void runVelocityMAXMotion(double velocity) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setFF(double kS, double kG, double kV, double kA) {}

  default void stop() {}

  default void setBrakeMode(boolean enabled) {}
}
