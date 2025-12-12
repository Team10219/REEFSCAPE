// Copyright (c) 2025 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MechanicalAdvantage.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private static final LoggedTunableNumber Station = new LoggedTunableNumber("Elevator/Station", 0);
  private static final LoggedTunableNumber L1 = new LoggedTunableNumber("Elevator/L1", 9);
  private static final LoggedTunableNumber L2 = new LoggedTunableNumber("Elevator/L2", 18);
  private static final LoggedTunableNumber L3 = new LoggedTunableNumber("Elevator/L3", 29);
  private static final LoggedTunableNumber L4 = new LoggedTunableNumber("Elevator/L4", 45);
  private static final LoggedTunableNumber Hoop = new LoggedTunableNumber("Elevator/Hoop", 0);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/ClosedLoop/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/ClosedLoop/kI");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/ClosedLoop/kD");

  static {
    switch (Constants.getRobot()) {
      case COMPBOT -> {
        kP.initDefault(0.065);
        kI.initDefault(0.000007);
        kD.initDefault(0.24);
      }
      case SIMBOT -> {
        kP.initDefault(0.065);
        kI.initDefault(0.000007);
        kD.initDefault(0.24);
      }
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get());
    }
  }

  public Command setPosition(double position) {
    return Commands.run(() -> io.setPosition(position));
  }

  public Command setVolts(double volts) {
    return Commands.runEnd(() -> io.runVolts(volts), () -> io.stop());
  }

  public Command goToStation() {
    return Commands.runOnce(() -> io.setPosition(Station.get()));
  }

  public Command goToL1() {
    return Commands.runOnce(() -> io.setPosition(L1.get()));
  }

  public Command goToL2() {
    return Commands.runOnce(() -> io.setPosition(L2.get()));
  }

  public Command goToL3() {
    return Commands.runOnce(() -> io.setPosition(L3.get()));
  }

  public Command goToL4() {
    return Commands.runOnce(() -> io.setPosition(L4.get()));
  }

  public Command goToHoop() {
    return Commands.runOnce(() -> io.setPosition(Hoop.get()));
  }
}
