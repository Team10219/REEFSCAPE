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
  private CommandXboxController controller;

  public ControlsXbox(int port) {
    controller = new CommandXboxController(port);
  }

  @Override
  public double getForward() {
    return AllianceFlipUtil.shouldFlip() ? controller.getLeftY() : -controller.getLeftY();
  }

  @Override
  public double getStrafe() {
    return AllianceFlipUtil.shouldFlip() ? controller.getLeftX() : -controller.getLeftX();
  }

  @Override
  public double getTurn() {
    return -controller.getRightX();
  }

  @Override
  public Trigger resetFieldCentric() {
    return controller.start();
  }

  @Override
  public Trigger lockToZero() {
    return controller.back();
  }

  @Override
  public Trigger xWheels() {
    return controller.x();
  }

  @Override
  public Trigger reefFace() {
    return controller.b();
  }

  @Override
  public Trigger autoAlign(Boolean right) {
    return right ? controller.rightBumper() : controller.leftBumper();
  }

  @Override
  public Trigger Source() {
    return controller.povUp();
  }

  @Override
  public Trigger Level1() {
    return controller.povLeft();
  }

  @Override
  public Trigger Level2() {
    return controller.povDown();
  }

  @Override
  public Trigger Level3() {
    return controller.povRight();
  }

  @Override
  public Trigger Level4() {
    return null; // We can't do L4 rn
  }

  @Override
  public Trigger hoopHeight() {
    return controller.x();
  }

  @Override
  public Trigger Intake() {
    return controller.leftTrigger();
  }

  @Override
  public Trigger Spit() {
    return controller.rightTrigger();
  }
}
