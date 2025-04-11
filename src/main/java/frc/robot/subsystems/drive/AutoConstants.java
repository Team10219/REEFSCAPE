// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

/** Add your docs here. */
public class AutoConstants {
  public static final double X_REEF_ALIGN_P = 1.5;
  public static final double X_REEF_ALIGN_I = 0;
  public static final double X_REEF_ALIGN_D = 0;

  public static final double Y_REEF_ALIGN_P = 1.5;
  public static final double Y_REEF_ALIGN_I = 0;
  public static final double Y_REEF_ALIGN_D = 0;

  public static final double R_REEF_ALIGN_P = 0.1;
  public static final double R_REEF_ALIGN_I = 0;
  public static final double R_REEF_ALIGN_D = 0;

  public static final double R_SETPOINT_REEF_ALIGNMENT = 0; // Rotation
  public static final double R_TOLERANCE_REEF_ALIGNMENT = 1;
  public static final double X_SETPOINT_REEF_ALIGNMENT = -0.34; // Vertical pose
  public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
  public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16; // Horizontal pose
  public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

  public static final double DONT_SEE_TAG_WAIT_TIME = 1;
  public static final double POSE_VALIDATION_TIME = 0.3;
}
