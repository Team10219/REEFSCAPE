// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
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

  private double gearing = 5/1;
  private double leftAppliedVoltage = 0.0;
  private double rightAppliedVoltage = 0.0;

  public IntakeIOSim() {
    leftSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(leftGearbox, 0, gearing), leftGearbox);
    rightSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(rightGearbox, 0, gearing), rightGearbox);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0.0);
    }

    leftSim.update(Constants.loopPeriodSecs);
    rightSim.update(Constants.loopPeriodSecs);

    inputs.leftConnected = true;
    inputs.leftPositionRads = leftSim.getAngularPositionRad();
    inputs.leftVelocityRadPerSec = leftSim.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = leftAppliedVoltage;
    inputs.leftCurrentAmps = leftSim.getCurrentDrawAmps();
    inputs.leftTempCelsius = 0.0;

    inputs.rightConnected = true;
    inputs.rightPositionRads = rightSim.getAngularPositionRad();
    inputs.rightVelocityRadPerSec = rightSim.getAngularVelocityRadPerSec();
    inputs.rightAppliedVolts = rightAppliedVoltage;
    inputs.rightCurrentAmps = rightSim.getCurrentDrawAmps();
    inputs.rightTempCelsius = 0.0;
  }

  @Override
  public void runVolts(double volts) {
    leftAppliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    rightAppliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);

    leftSim.setInputVoltage(leftAppliedVoltage);
    rightSim.setInputVoltage(rightAppliedVoltage);
  }

  @Override
  public void runSeperateVolts(double leftVolts, double rightVolts) {
    leftAppliedVoltage = MathUtil.clamp(leftVolts, -12.0, 12.0);
    rightAppliedVoltage = MathUtil.clamp(rightVolts, -12.0, 12.0);

    leftSim.setInputVoltage(leftAppliedVoltage);
    rightSim.setInputVoltage(rightAppliedVoltage);
  }
}
