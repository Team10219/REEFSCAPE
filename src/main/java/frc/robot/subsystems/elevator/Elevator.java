// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private static final LoggedTunableNumber Station = new LoggedTunableNumber("Elevator/Station", 0);
  private static final LoggedTunableNumber L1 = new LoggedTunableNumber("Elevator/L1", 9);
  private static final LoggedTunableNumber L2 = new LoggedTunableNumber("Elevator/L2", 18);
  private static final LoggedTunableNumber L3 = new LoggedTunableNumber("Elevator/L3", 29);
  private static final LoggedTunableNumber L4 = new LoggedTunableNumber("Elevator/L4", 45);

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public Command setPosition(double position) {
    return Commands.run(() -> io.setPosition(position));
  }

  /**
   * Why dont I just use setposition for everything? Simple, it would make it look ugly in
   * RobotContainer, and i want to maintain readability
   */
  public Command goToStation() {
    return Commands.run(() -> io.setPosition(Station.get()));
  }

  public Command goToL1() {
    return Commands.run(() -> io.setPosition(L1.get()));
  }

  public Command goToL2() {
    return Commands.run(() -> io.setPosition(L2.get()));
  }

  public Command goToL3() {
    return Commands.run(() -> io.setPosition(L3.get()));
  }

  public Command goToL4() {
    return Commands.run(() -> io.setPosition(L4.get()));
  }
}
