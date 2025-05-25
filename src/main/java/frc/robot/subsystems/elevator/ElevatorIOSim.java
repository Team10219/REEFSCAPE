package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorIOSim implements ElevatorIO {
  private final PIDController pid;
  private boolean usingPID = false;
  private double position = 0.0;
  private double speed = 0.0;
  private double target = 0.0;

  public ElevatorIOSim() {
    pid = new PIDController(0.01, 0, 0);
  }

  @Override
  public void setPower(double power) {
    speed = power;
    usingPID = false;
  }

  @Override
  public void setVoltage(double volts) {
    setPower(volts / 12);
  }

  @Override
  public void setPosition(double encoderPosition) {
    target = encoderPosition;
    usingPID = true;
  }

  @Override
  public void update(ElevatorIOInputs inputs) {

    position += speed * 8;

    if (usingPID) {
      speed = pid.calculate(position, target);
      speed = Math.max(-1, Math.min(1, speed));
    }

    inputs.position = position;
    inputs.velocity = speed * 8; // Simulated velocity
    inputs.target = target;
  }

  @Override
  public void stop() {
    speed = 0.0;
  }

  @Override
  public void zero() {
    position = 0.0;
  }

  @Override
  public void brakeMode(boolean brakeEnabled) {}
}
