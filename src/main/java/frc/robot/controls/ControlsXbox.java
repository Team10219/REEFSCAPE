// Copyright (c) 2025 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.MechanicalAdvantage.AllianceFlipUtil;

/** Add your docs here. */
public class ControlsXbox implements Controls {
  private CommandXboxController driver;

  public ControlsXbox(int port) {
    driver = new CommandXboxController(port);
  }

  @Override
  public double getForward() {
    return AllianceFlipUtil.shouldFlip() ? driver.getLeftY() : -driver.getLeftY();
  }

  @Override
  public double getStrafe() {
    return AllianceFlipUtil.shouldFlip() ? driver.getLeftX() : -driver.getLeftX();
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

  @Override
  public Trigger Source() {
    return driver.povUp();
  }

  @Override
  public Trigger Level1() {
    return driver.povLeft();
  }

  @Override
  public Trigger Level2() {
    return driver.povDown();
  }

  @Override
  public Trigger Level3() {
    return driver.povRight();
  }

  @Override
  public Trigger Level4() {
    return null; // We can't do L4 rn
  }

  @Override
  public Trigger Intake() {
    return driver.leftTrigger();
  }

  @Override
  public Trigger Spit() {
    return driver.rightTrigger();
  }
}
