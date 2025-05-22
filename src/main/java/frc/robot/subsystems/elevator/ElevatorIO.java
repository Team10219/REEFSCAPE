// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    boolean leaderConnected;
    boolean followerConnected;
    double leaderTemp;
    double followerTemp;
    double position;
    double velocity;
    double target;
    boolean bottomLimit = false;
  }

  public void update(ElevatorIOInputs inputs);

  public void setPower(double power);

  public void setVoltage(double voltage);

  public void setPosition(double encoderValue);

  public void stop();

  public void zero();

  public void brakeMode(boolean brakeEnabled);

  public default void simulationPeriodic() {}
}
