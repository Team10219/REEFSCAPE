// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorEncoderSetpoints;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorValues;
import frc.robot.subsystems.elevator.ElevatorConstants.Setpoint;
import frc.robot.util.SparkConfigs.ElevatorConfigs;
import frc.robot.util.SparkUtil;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax leader, follower;
  private SparkClosedLoopController elevatorClosedLoopController;
  private RelativeEncoder elevatorEncoder;
  private RelativeEncoder followerEncoder;

  // private DigitalInput limitSwitch = new DigitalInput(0);

  private boolean wasResetByLimit = false;
  private double elevatorCurrentTarget =
      ElevatorEncoderSetpoints.kSource; // Set Defualt Resting Postion

  public ElevatorSubsystem() {
    leader = new SparkMax(ElevatorValues.kLeaderCAN, MotorType.kBrushless);
    follower = new SparkMax(ElevatorValues.kFollowerCAN, MotorType.kBrushless);
    elevatorClosedLoopController = leader.getClosedLoopController();
    elevatorEncoder = leader.getEncoder();
    followerEncoder = follower.getEncoder();

    // Apply SparkConfigs
    SparkUtil.tryUntilOk(
        leader,
        5,
        () ->
            leader.configure(
                ElevatorConfigs.elevatorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(
        follower,
        5,
        () ->
            follower.configure(
                ElevatorConfigs.elevatorConfig.follow(ElevatorValues.kLeaderCAN, true),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    elevatorEncoder.setPosition(0);
  }

  public void goToSetpoint() {
    elevatorClosedLoopController.setReference(
        elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kSoruce:
              elevatorCurrentTarget = ElevatorEncoderSetpoints.kSource;
              break;
            case kLevel1:
              elevatorCurrentTarget = ElevatorEncoderSetpoints.kLevel1;
              break;
            case kLevel2:
              elevatorCurrentTarget = ElevatorEncoderSetpoints.kLevel2;
              break;
            case kLevel3:
              elevatorCurrentTarget = ElevatorEncoderSetpoints.kLevel3;
              break;
            case kLevel4:
              elevatorCurrentTarget = ElevatorEncoderSetpoints.kLevel4;
              break;
            default:
              elevatorCurrentTarget = ElevatorEncoderSetpoints.kSource;
              break;
          }
        });
  }

  public Command resetEncodersOnButtonPress() {
    return this.runOnce(
        () -> {
          elevatorEncoder.setPosition(0);
        });
  }

  // Spark Limit Switches
  public void resetEncoderOnLimitSwitch() {
    if (!wasResetByLimit
        && (leader.getForwardLimitSwitch().isPressed()
            || follower.getForwardLimitSwitch().isPressed())) {
      elevatorEncoder.setPosition(0);
      followerEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!leader.getForwardLimitSwitch().isPressed()
        || !follower.getForwardLimitSwitch().isPressed()) {
      wasResetByLimit = false;
    }
  }

  @Override
  public void periodic() {
    goToSetpoint();
    resetEncoderOnLimitSwitch();

    SmartDashboard.putNumber("Elevator Target", elevatorCurrentTarget);
    SmartDashboard.putNumber("Elevator Postion", elevatorEncoder.getPosition());
    SmartDashboard.putBoolean("Forward LimitSwitch", follower.getForwardLimitSwitch().isPressed());
    SmartDashboard.putBoolean("Forward LimitSwitch2", leader.getForwardLimitSwitch().isPressed());
    SmartDashboard.putBoolean("wasResetByLimit", wasResetByLimit);
  }
}
