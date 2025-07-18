// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.canID.intakeID.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class IntakeIOSimSpark implements IntakeIO {
  private final SparkMax leftSpark;
  private final SparkMaxSim leftSim;
  private final SparkRelativeEncoderSim leftEncoder;
  private ControlType leftControlType = null;

  private final SparkMax rightSpark;
  private final SparkMaxSim rightSim;
  private final SparkRelativeEncoderSim rightEncoder;
  private ControlType rightControlType = null;

  private double maxAcceleration = 10000;
  private double maxVelocity = 4000;
  private double Kv = 473;
  private int currentLimit = 50;
  private int freeLimit = 40;
  private boolean brakeModeEnabled = true;

  public IntakeIOSimSpark() {
    leftSpark = new SparkMax(left, MotorType.kBrushless);
    leftSim = new SparkMaxSim(leftSpark, DCMotor.getNEO(1));
    leftEncoder = leftSim.getRelativeEncoderSim();

    rightSpark = new SparkMax(right, MotorType.kBrushless);
    rightSim = new SparkMaxSim(rightSpark, DCMotor.getNEO(1));
    rightEncoder = rightSim.getRelativeEncoderSim();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.leftConnected = true;
    inputs.leftPositionRads = leftEncoder.getPosition();
    inputs.leftVelocityRadPerSec = leftEncoder.getVelocity();
    inputs.leftAppliedVolts = leftSim.getBusVoltage() * leftSim.getAppliedOutput();
    inputs.leftCurrentAmps = leftSim.getMotorCurrent();
    inputs.leftTempCelsius = 0.0;
    inputs.leftControlType = leftControlType != null ? leftControlType.toString() : "None";

    inputs.rightConnected = true;
    inputs.rightPositionRads = rightEncoder.getPosition();
    inputs.rightVelocityRadPerSec = rightEncoder.getVelocity();
    inputs.rightAppliedVolts = rightSim.getBusVoltage() * rightSim.getAppliedOutput();
    inputs.rightCurrentAmps = rightSim.getMotorCurrent();
    inputs.rightTempCelsius = 0.0;
    inputs.rightControlType = rightControlType != null ? rightControlType.toString() : "None";
  }

  @Override
  public void runOpenLoop(double output) {
    leftSim.setAppliedOutput(output);
    rightSim.setAppliedOutput(output);

    leftControlType = ControlType.kDutyCycle;
    rightControlType = ControlType.kDutyCycle;
  }

  @Override
  public void runVelocity(double velocity) {
    leftSim.setVelocity(velocity);
    rightSim.setVelocity(velocity);

    leftControlType = ControlType.kVelocity;
    rightControlType = ControlType.kVelocity;
  }

}
