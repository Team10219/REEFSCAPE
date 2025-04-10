// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignCommand extends Command {

  private PIDController xController, yController, rController;
  private boolean isRightBranch;
  private Timer dontSeeTagTimer, stopTimer;
  private Drive m_drivetrain;
  private double tagID = -1;

  public AutoAlignCommand(boolean isRightBranch, Drive m_drivetrain) {
    xController =
        new PIDController(
            AutoConstants.X_REEF_ALIGN_P,
            AutoConstants.X_REEF_ALIGN_I,
            AutoConstants.X_REEF_ALIGN_D);
    yController =
        new PIDController(
            AutoConstants.Y_REEF_ALIGN_P,
            AutoConstants.Y_REEF_ALIGN_I,
            AutoConstants.Y_REEF_ALIGN_D);
    rController =
        new PIDController(
            AutoConstants.R_REEF_ALIGN_P,
            AutoConstants.R_REEF_ALIGN_I,
            AutoConstants.R_REEF_ALIGN_D);

    this.isRightBranch = isRightBranch;
    this.m_drivetrain = m_drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rController.setSetpoint(AutoConstants.R_SETPOINT_REEF_ALIGNMENT);
    rController.setTolerance(AutoConstants.R_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(AutoConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(AutoConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(AutoConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(AutoConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", positions[2]);

      double xSpeed = xController.calculate(positions[2]);
      SmartDashboard.putNumber("xSpeed", xSpeed);

      double ySpeed = -yController.calculate(positions[0]);
      double rValue = -rController.calculate(positions[4]);

      m_drivetrain.setControl(xSpeed, ySpeed, rValue);

      if (!rController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      m_drivetrain.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.dontSeeTagTimer.hasElapsed(AutoConstants.DONT_SEE_TAG_WAIT_TIME)
        || stopTimer.hasElapsed(AutoConstants.POSE_VALIDATION_TIME);
  }
}
