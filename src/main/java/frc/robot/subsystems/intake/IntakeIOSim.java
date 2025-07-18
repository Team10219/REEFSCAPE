// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Literally just completely stolen from 6328, thanks! */
public class IntakeIOSim implements IntakeIO {

  private final DCMotorSim leftSim;
  private final DCMotorSim rightSim;
  private final DCMotor leftGearbox = DCMotor.getNEO(1);
  private final DCMotor rightGearbox = DCMotor.getNEO(1);

  private boolean leftClosedLoop = false;
  private boolean rightClosedLoop = false;
  private PIDController leftController = new PIDController(0.1, 0, 0);
  private PIDController rightController = new PIDController(0.1, 0, 0);
  private ControlType leftControlType = null;
  private ControlType rightControlType = null;

  private final double simKv = 1 / 473;
  private final double simKs = 0.0;

  private double gearing = 5 / 1;
  private double MOI = 0.00024;

  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;
  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  public IntakeIOSim() {
    leftSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(leftGearbox, MOI, gearing), leftGearbox);
    rightSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(rightGearbox, MOI, gearing), rightGearbox);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    if (leftClosedLoop) {
      leftAppliedVolts = leftController.calculate(leftSim.getAngularVelocityRadPerSec());
    } else {
      leftController.reset();
    }
    if (rightClosedLoop) {
      rightAppliedVolts = rightController.calculate(rightSim.getAngularVelocityRadPerSec());
    } else {
      rightController.reset();
    }

    if (DriverStation.isDisabled()) {
      runVolts(0.0);
    }

    leftSim.setInputVoltage(MathUtil.clamp(leftAppliedVolts, -12.0, 12.0));
    rightSim.setInputVoltage(MathUtil.clamp(rightAppliedVolts, -12.0, 12.0));
    leftSim.update(Constants.loopPeriodSecs);
    rightSim.update(Constants.loopPeriodSecs);

    inputs.leftConnected = true;
    inputs.leftPositionRads = leftSim.getAngularPositionRad();
    inputs.leftVelocityRadPerSec = leftSim.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = Math.abs(leftSim.getCurrentDrawAmps());
    inputs.leftTempCelsius = 0.0;
    inputs.leftControlType = leftControlType.toString();

    inputs.rightConnected = true;
    inputs.rightPositionRads = rightSim.getAngularPositionRad();
    inputs.rightVelocityRadPerSec = rightSim.getAngularVelocityRadPerSec();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = Math.abs(rightSim.getCurrentDrawAmps());
    inputs.rightTempCelsius = 0.0;
    inputs.rightControlType = rightControlType.toString();
  }

  @Override
  public void runOpenLoop(double output) {
    leftClosedLoop = false;
    rightClosedLoop = false;

    leftAppliedVolts = output;
    rightAppliedVolts = output;
  }

  @Override
  public void runVolts(double volts) {
    leftClosedLoop = false;
    rightClosedLoop = false;

    leftAppliedVolts = volts;
    rightAppliedVolts = volts;
  }

  @Override
  public void runSeparateVolts(double leftVolts, double rightVolts) {
    leftClosedLoop = false;
    rightClosedLoop = false;

    leftAppliedVolts = leftVolts;
    rightAppliedVolts = rightVolts;
  }

  @Override
  public void runVelocity(double velocity) {
    leftClosedLoop = true;
    rightClosedLoop = true;

    leftFFVolts = simKs * Math.signum(velocity) + simKv * velocity;
    rightFFVolts = simKs * Math.signum(velocity) + simKv * velocity;

    leftController.setSetpoint(velocity);
    rightController.setSetpoint(velocity);
  }

  @Override
  public void runVelocityMAXMotion(double velocity) {
    leftClosedLoop = true;
    rightClosedLoop = true;

    leftControlType = ControlType.kMAXMotionVelocityControl;
    rightControlType = ControlType.kMAXMotionVelocityControl;

    leftFFVolts = simKs * Math.signum(velocity) + simKv * velocity;
    rightFFVolts = simKs * Math.signum(velocity) + simKv * velocity;

    leftController.setSetpoint(velocity);
    rightController.setSetpoint(velocity);
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
