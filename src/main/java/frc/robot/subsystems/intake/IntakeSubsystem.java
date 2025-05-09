// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeCans;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSpeeds;
import frc.robot.util.SparkConfigs.IntakeConfigs;
import frc.robot.util.SparkUtil;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  SparkMax intakeLeft, intakeRight;

  public IntakeSubsystem() {
    intakeLeft = new SparkMax(IntakeCans.kLeftIntakeCAN, MotorType.kBrushless); // Leader
    intakeRight = new SparkMax(IntakeCans.kRightIntakeCAN, MotorType.kBrushless);

    SparkUtil.tryUntilOk(
        intakeLeft,
        5,
        () ->
            intakeLeft.configure(
                IntakeConfigs.intakeConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(
        intakeRight,
        5,
        () ->
            intakeRight.configure(
                IntakeConfigs.intakeConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  public void setIntakePower(double speed) {
    intakeLeft.set(speed);
    intakeRight.set(speed * -1);
  }

  public Command Intake() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSpeeds.kIntakeSpeed), () -> this.setIntakePower(0));
  }

  public Command Outtake() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSpeeds.kOuttakeSpeed), () -> this.setIntakePower(0));
  }

  public Command Grab() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSpeeds.kGrab), () -> this.setIntakePower(0));
  }

  public Command Stop() {
    return this.startEnd(() -> this.setIntakePower(0), () -> this.setIntakePower(0));
  }

  public Command SpitL1() {
    return this.startRun(
        () -> this.intakeLeft.set(IntakeSpeeds.kIntakeSpeed),
        () -> this.intakeRight.set(IntakeSpeeds.kL1Speed));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Speed", intakeLeft.get());
  }
}
