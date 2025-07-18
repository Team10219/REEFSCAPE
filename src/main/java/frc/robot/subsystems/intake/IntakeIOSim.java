// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.canID.intakeID.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/** Add your docs here. */
public class IntakeIOSim extends IntakeIOSpark {

  private final SparkMaxSim leftSparkSim;
  private final SparkRelativeEncoderSim leftEncoderSim;
  private final DCMotor leftGearbox;
  private final FlywheelSim leftRollerSim;

  private final SparkMaxSim rightSparkSim;
  private final SparkRelativeEncoderSim rightEncoderSim;
  private final DCMotor rightGearbox;
  private final FlywheelSim rightRollerSim;

  private final double gearing = 5;
  private final double moi = 0.00024;

  private SparkMaxConfig simConfig;

  public IntakeIOSim() {
    super();

    simConfig = new SparkMaxConfig();
    simConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(0.1).i(0.0).d(0.0);

    leftSparkSim = new SparkMaxSim(leftSpark, DCMotor.getNEO(1));
    leftEncoderSim = leftSparkSim.getRelativeEncoderSim();
    leftGearbox = DCMotor.getNEO(1);
    leftRollerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(leftGearbox, moi, gearing), leftGearbox);

    rightSparkSim = new SparkMaxSim(rightSpark, DCMotor.getNEO(1));
    rightEncoderSim = rightSparkSim.getRelativeEncoderSim();
    rightGearbox = DCMotor.getNEO(1);
    rightRollerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(rightGearbox, moi, gearing), rightGearbox);

    tryUntilOk(
        leftSpark,
        5,
        () ->
            leftSpark.configure(
                simConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        rightSpark,
        5,
        () ->
            rightSpark.configure(
                simConfig.follow(left, true),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    leftRollerSim.setInputVoltage(leftSpark.getBusVoltage() * leftSpark.getAppliedOutput());
    leftRollerSim.update(Constants.loopPeriodSecs);
    leftSparkSim.iterate(
        leftRollerSim.getAngularVelocityRadPerSec(), 12.0, Constants.loopPeriodSecs);

    rightRollerSim.setInputVoltage(rightSpark.getBusVoltage() * rightSpark.getAppliedOutput());
    rightRollerSim.update(Constants.loopPeriodSecs);
    rightSparkSim.iterate(
        rightRollerSim.getAngularVelocityRadPerSec(), 12.0, Constants.loopPeriodSecs);

    super.updateInputs(inputs);
  }
}
