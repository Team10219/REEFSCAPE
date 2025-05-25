// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class DriverControlsXbox implements DriverControls {
  private CommandXboxController driver;

  public DriverControlsXbox(int port) {
    driver = new CommandXboxController(port);
  }

  @Override
  public double getForward() {
    return -driver.getLeftY();
  }

  @Override
  public double getStrafe() {
    return -driver.getLeftX();
  }

  @Override
  public double getTurn() {
    return -driver.getRightX();
  }

  @Override
  public Trigger resetFieldCentric() {
    return driver.start();
  }

  @Override
  public Trigger lockToZero() {
    return driver.back();
  }

  @Override
  public Trigger xWheels() {
    return driver.x();
  }

  @Override
  public Trigger reefFace() {
    return driver.b();
  }

  @Override
  public Trigger autoAlign(Boolean right) {
    return right ? driver.rightBumper() : driver.leftBumper();
  }
}
