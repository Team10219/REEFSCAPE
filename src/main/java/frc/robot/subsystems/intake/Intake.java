// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;


import static com.revrobotics.spark.SparkBase.ControlType.*;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.update(inputs);
    Logger.processInputs("Intake", inputs);
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  public Command setVeloctiy(double leftVel, double rightVel) {
    return Commands.runEnd(
        () ->
            io.setVelocity(
                leftVel,
                ControlType.kMAXMotionVelocityControl,
                rightVel,
                ControlType.kMAXMotionVelocityControl),
        () -> io.stop());
  }

  public Command setPower(double leftPower, double rightPower) {
    return Commands.runEnd(() -> io.setPower(leftPower, rightPower), () -> io.stop());
  }

  public Command setVoltage(double leftVolts, double rightVolts) {
    return Commands.runEnd(() -> io.setVoltage(leftVolts, rightVolts), () -> io.stop());
  }

  public Command Intake() {
    return Commands.runEnd(() -> io.setVelocity(0, kMAXMotionVelocityControl, 0, kMAXMotionVelocityControl), () -> io.stop());
  }
}
