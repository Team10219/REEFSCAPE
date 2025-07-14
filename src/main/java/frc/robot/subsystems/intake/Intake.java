// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Our intake is two motors, not mechanically connected in any way, they run completely individually
 * of eachother
 */
public class Intake extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP", 0.1);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake/kI", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD", 0.0);
  private static final LoggedTunableNumber vF = new LoggedTunableNumber("Intake/vF", 0.0);

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command runOpenLoop(double output) {
    return Commands.runEnd(() -> io.runOpenLoop(output), () -> io.stop());
  }

  public Command runVolts(double volts) {
    return Commands.runEnd(() -> io.runVolts(volts), () -> io.stop());
  }

  public Command runSeperateVolts(double leftVolts, double rightVolts) {
    return Commands.runEnd(() -> io.runSeperateVolts(leftVolts, rightVolts), () -> io.stop());
  }

  public Command runVelocity(double velocity) {
    return Commands.sequence(
        Commands.runOnce(() -> io.setPIDV(kP.get(), kI.get(), kD.get(), vF.get())),
        Commands.runEnd(() -> io.runVelocity(velocity), () -> io.stop()));
  }

  public Command runVelocityMAXMotion(double velocity) {
    return Commands.sequence(
        Commands.runOnce(() -> io.setPIDV(kP.get(), kI.get(), kD.get(), vF.get())),
        Commands.runEnd(() -> io.runVelocityMAXMotion(velocity), () -> io.stop()));
  }
}
