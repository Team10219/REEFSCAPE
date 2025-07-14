// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Literally just completely stolen from 6328, thanks! */
public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim leftSim;
  private final DCMotorSim rightSim;
  private final DCMotor gearbox;
  private double leftAppliedVoltage = 0.0;
  private double rightAppliedVoltage = 0.0;

  public IntakeIOSim(DCMotor motorModel, double reduction, double moi) {
    gearbox = motorModel;
    leftSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
    rightSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
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
  }
}
