// Copyright (c) 2025 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MechanicalAdvantage.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Our intake is two motors, not mechanically connected in any way, they run completely individually
 * of eachother
 */
public class Intake extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP", 0.1);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake/kI", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD", 0.0);

  static {
    switch (Constants.getRobot()) {
      case COMPBOT -> {
        kP.initDefault(0.1);
        kI.initDefault(0.0);
        kD.initDefault(0.0);
      }
      case SIMBOT -> {
        kP.initDefault(0.1);
        kI.initDefault(0.0);
        kD.initDefault(0.0);
      }
    }
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Intake", inputs);

    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get());
    }
  }

  public Command Thru() {
    return Commands.runEnd(() -> io.runOpenLoop(-0.1), () -> io.stop());
  }

  public Command Spit() {
    return Commands.runEnd(() -> io.runOpenLoop(-0.45), () -> io.stop());
  }

  public Command setVelocity(double value) {
    return Commands.runEnd(() -> io.runVelocity(value), () -> io.stop());
  }
}
