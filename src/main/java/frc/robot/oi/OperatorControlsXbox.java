// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class OperatorControlsXbox implements OperatorControls {
  private CommandXboxController operator;

  public OperatorControlsXbox(int port) {
    operator = new CommandXboxController(port);
  }

  @Override
  public Trigger Source() {
    return operator.povUp();
  }

  @Override
  public Trigger Level1() {
    return operator.povLeft();
  }

  @Override
  public Trigger Level2() {
    return operator.povDown();
  }

  @Override
  public Trigger Level3() {
    return operator.povRight();
  }

  @Override
  public Trigger Level4() {
    return null; // We can't do L4 rn
  }

  @Override
  public Trigger Intake() {
    return operator.leftTrigger();
  }

  @Override
  public Trigger Spit() {
    return operator.rightTrigger();
  }
}
