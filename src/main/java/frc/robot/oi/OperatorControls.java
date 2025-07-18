// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public interface OperatorControls {
  public Trigger Source();

  public Trigger Level1();

  public Trigger Level2();

  public Trigger Level3();

  public Trigger Level4();

  public Trigger Intake();

  public Trigger Spit();
}
