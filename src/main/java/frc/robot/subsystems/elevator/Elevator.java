// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private static final double inPositionTolerance = 0.01; // Meters

  public final LoggedTunableNumber Source = new LoggedTunableNumber("Elevator/Source", 0);
  public final LoggedTunableNumber Level1 = new LoggedTunableNumber("Elevator/Level1", 9);
  public final LoggedTunableNumber Level2 = new LoggedTunableNumber("Elevator/Level2", 18);
  public final LoggedTunableNumber Level3 = new LoggedTunableNumber("Elevator/Level3", 29);
  public final LoggedTunableNumber Level4 = new LoggedTunableNumber("Elevator/Level4", 45);

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // @AutoLogOutput(key = "Elevator/TargetPosition")
  // private double targetPosition = 0.0;

  @AutoLogOutput(key = "Elevator/TargetPosition")
  private double targetPosition = inputs.target;

  @AutoLogOutput private boolean hasHomed = false;

  private Trigger atLowerLimitDebounced = new Trigger(this::isAtLowerLimit).debounce(0.3);

  /**
   * Creates a new ElevatorSubsystem.
   *
   * @param elevatorIOSpark
   */
  public Elevator(ElevatorIO io) {
    this.io = io;
    io.zero();
  }

  @Override
  public void periodic() {
    io.update(inputs);
    Logger.processInputs("Elevator", inputs);

    // Homing logic
    if (isAtLowerLimit() && Math.abs(inputs.position) <= 0.001) {
      hasHomed = true;
    }

    isElevatorInPosition();

    // Log target and actual positions
    Logger.recordOutput("Elevator/TargetPosition", targetPosition);
    Logger.recordOutput("Elevator/CurrentPosition", inputs.position);
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  public double getPositionMeters() {
    return inputs.position;
  }

  public Command runSpeed(double speed) {
    return Commands.runEnd(() -> io.setPower(speed), () -> io.stop())
        .withName("Run Speed " + speed);
  }

  public Command brakeMode(boolean enabled) {
    return Commands.runOnce(() -> io.brakeMode(enabled));
  }

  public Command stop() {
    return Commands.run(() -> io.stop()).withName("stop");
  }

  public Command home() {
    return runOnce(() -> hasHomed = false)
        .andThen(
            Commands.waitUntil(this::isAtLowerLimit)
                .andThen(
                    Commands.runOnce(
                        () -> {
                          io.zero();
                          io.stop();
                          inputs.target = 0;
                        }))
                .deadlineFor(runSpeed(-0.03)));
  }

  public Command stow() {
    return setPosition(Source.getAsDouble()).until(atLowerLimitDebounced).withName("stow");
  }

  public Command zero() {
    return setPosition(0).withName("zero");
  }

  public Command setPosition(double position) {
    return run(() -> {
          targetPosition = position;
          io.setPosition(position);
        })
        .withName("Set Position");
  }

  public boolean isAtLowerLimit() {
    return inputs.bottomLimit;
  }

  @AutoLogOutput(key = "Elevator/elevatorAtTarget")
  public boolean isElevatorInPosition() {
    return Math.abs(targetPosition - inputs.position) <= inPositionTolerance;
  }
}
