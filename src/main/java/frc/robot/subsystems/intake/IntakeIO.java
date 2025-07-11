// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    boolean leftConnected = false;
    boolean rightConnected = false;
    double leftTemp = 0.0;
    double rightTemp = 0.0;
    double leftVelocityRPM = 0.0;
    double rightVelocityRPM = 0.0;
  }

  public default void update(IntakeIOInputs inputs) {}

  public default void setVelocity(
      double leftVel,
      SparkBase.ControlType controlTypeLeft,
      double rightVel,
      SparkBase.ControlType controlTypeRight) {}

  public default void setControlType(SparkBase.ControlType left, SparkBase.ControlType right) {}

  public default void setPower(double leftPower, double rightPower) {}

  public default void setVoltage(double leftVolts, double rightVolts) {}

  public default void stop() {}

  public default void brakeMode(boolean enabled) {}

  public default void simulationPeriodic() {}
}
