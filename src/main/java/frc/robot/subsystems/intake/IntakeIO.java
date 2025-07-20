// Copyright (c) 2025 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean leftConnected = false;
    public MutAngularVelocity leftAngularVelocityDPS = DegreesPerSecond.mutable(0);
    public MutVoltage leftAppliedVolts = Volts.mutable(0);
    public MutCurrent leftCurrentAmps = Amps.mutable(0);
    public MutTemperature leftTempCelsius = Celsius.mutable(0);
    public double hello = 0.0;
    public String leftControlType = null;

    public boolean rightConnected = false;
    public MutAngularVelocity rightAngularVelocityDPS = DegreesPerSecond.mutable(0);
    public MutVoltage rightAppliedVolts = Volts.mutable(0);
    public MutCurrent rightCurrentAmps = Amps.mutable(0);
    public MutTemperature rightTempCelsius = Celsius.mutable(0);
    public String rightControlType = null;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(Voltage volts) {}

  default void runVelocity(AngularVelocity velocity) {}

  default void runVelocityMAXMotion(AngularVelocity velocity) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setFF(double kS, double kG, double kV, double kA) {}

  default void stop() {}

  default void setBrakeMode(boolean enabled) {}
}
