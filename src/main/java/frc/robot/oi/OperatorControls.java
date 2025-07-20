// Copyright (c) 2025 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

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
