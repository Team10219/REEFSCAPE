package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorIOSim implements ElevatorIO {
  private final PIDController pid;
  private boolean usingPID = false;
  private double position = 0.0;
  private double speed = 0.0;
  private double target = 0.0;

  public ElevatorIOSim() {
    pid = new PIDController(0.01, 0, 0); // PID gains for simulation
  }

  @Override
  public void setPower(double power) {
    speed = power;
    usingPID = false; // Disable PID control when directly setting power
  }

  @Override
  public void setVoltage(double volts) {
    setPower(volts / 12); // Convert voltage to power (-1 to 1 range)
  }

  @Override
  public void setPosition(double encoderPosition) {
    target = encoderPosition;
    usingPID = true; // Enable PID control for position
  }

  @Override
  public void update(ElevatorIOInputs inputs) {
    // Simulate movement with the current speed
    position += speed * 8; // Update position (scaled by time and factor)

    // If using PID control, calculate the new speed
    if (usingPID) {
      speed = pid.calculate(position, target);
      speed = Math.max(-1, Math.min(1, speed)); // Clamp speed to [-1, 1]
    }

    // Update inputs for logging and feedback
    inputs.position = position;
    inputs.velocity = speed * 8; // Simulated velocity
    inputs.target = target;
  }

  @Override
  public void stop() {
    speed = 0.0; // Stop the elevator
  }

  @Override
  public void zero() {
    position = 0.0; // Reset the position to zero
  }

  @Override
  public void brakeMode(boolean brakeEnabled) {}
}
