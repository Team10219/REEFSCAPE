// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    boolean leaderConnected = false;
    boolean followerConnected = false;
    double leaderTemp = 0.0;
    double followerTemp = 0.0;
    double position = 0.0;
    double velocity = 0.0;
    double target = 0.0;
    boolean bottomLimit = false;
  }

  public default void update(ElevatorIOInputs inputs) {}
  ;

  public default void setPower(double power) {}
  ;

  public default void setVoltage(double voltage) {}
  ;

  public default void setPosition(double encoderValue) {}
  ;

  public default void stop() {}
  ;

  public default void zero() {}
  ;

  public default void brakeMode(boolean brakeEnabled) {}
  ;

  public default void simulationPeriodic() {}
}
