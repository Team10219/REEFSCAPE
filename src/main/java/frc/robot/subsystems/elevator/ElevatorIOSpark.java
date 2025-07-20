// Copyright (c) 2025 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.canID.ElevatorID.*;
import static frc.robot.util.MechanicalAdvantage.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.TrackedController;
import java.util.function.DoubleSupplier;

/**
 * follower always follows leader, for the commands I only run the leader and I set the config for
 * the follower to run inverted the leader
 */
public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax leaderSpark;
  private final RelativeEncoder leaderEncoder;
  private final TrackedController elevatorController;
  private final SparkMax followerSpark;
  private final RelativeEncoder followerEncoder;

  private final SparkMaxConfig config;

  private double maxAcceleration = 10000;
  private double maxVelocity = 4000;
  private int currentLimit = 80;
  private int freeLimit = 70;
  private boolean brakeModeEnabled = true;

  public ElevatorIOSpark() {
    leaderSpark = new SparkMax(leader, MotorType.kBrushless);
    leaderEncoder = leaderSpark.getEncoder();

    elevatorController = new TrackedController(leaderSpark.getClosedLoopController());

    followerSpark = new SparkMax(follower, MotorType.kBrushless);
    followerEncoder = followerSpark.getEncoder();

    config = new SparkMaxConfig();
    config
        .idleMode(brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast)
        .smartCurrentLimit(currentLimit, freeLimit)
        .voltageCompensation(12.0);
    config.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(1)
        .i(0)
        .d(0)
        .maxMotion
        .maxAcceleration(maxAcceleration)
        .maxVelocity(maxVelocity);

    tryUntilOk(
        leaderSpark,
        5,
        () ->
            leaderSpark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        followerSpark,
        5,
        () ->
            followerSpark.configure(
                config.follow(leader, true),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sparkStickyFault = false;

    inputs.elevatorControlType =
        elevatorController.getControlType() != null
            ? elevatorController.getControlType().toString()
            : "None";
    // inputs.atSetpoint = elevatorController.atSetpoint().getAsBoolean();

    inputs.leaderPositionRads =
        ifOkOrDefault(leaderSpark, leaderEncoder::getPosition, inputs.leaderPositionRads);
    inputs.leaderVelocityRadsPerSec =
        ifOkOrDefault(leaderSpark, leaderEncoder::getVelocity, inputs.leaderVelocityRadsPerSec);
    inputs.leaderAppliedVolts =
        ifOkOrDefault(
            leaderSpark,
            new DoubleSupplier[] {leaderSpark::getBusVoltage, leaderSpark::getAppliedOutput},
            x -> x[0] * x[1],
            inputs.leaderAppliedVolts);
    inputs.leaderCurrentAmps =
        ifOkOrDefault(leaderSpark, leaderSpark::getOutputCurrent, inputs.leaderCurrentAmps);
    inputs.leaderTempCelsius =
        ifOkOrDefault(leaderSpark, leaderSpark::getMotorTemperature, inputs.leaderTempCelsius);

    inputs.followerPositionRads =
        ifOkOrDefault(followerSpark, followerEncoder::getPosition, inputs.followerPositionRads);
    inputs.followerVelocityRadsPerSec =
        ifOkOrDefault(
            followerSpark, followerEncoder::getVelocity, inputs.followerVelocityRadsPerSec);
    inputs.followerAppliedVolts =
        ifOkOrDefault(
            followerSpark,
            new DoubleSupplier[] {followerSpark::getBusVoltage, followerSpark::getAppliedOutput},
            x -> x[0] * x[1],
            inputs.followerAppliedVolts);
    inputs.followerCurrentAmps =
        ifOkOrDefault(followerSpark, followerSpark::getOutputCurrent, inputs.followerCurrentAmps);
    inputs.followerTempCelsius =
        ifOkOrDefault(
            followerSpark, followerSpark::getMotorTemperature, inputs.followerTempCelsius);
  }

  @Override
  public void runOpenLoop(double output) {
    leaderSpark.set(output);
  }

  @Override
  public void runVolts(double volts) {
    leaderSpark.setVoltage(volts);
  }

  @Override
  public void stop() {
    leaderSpark.stopMotor();
    followerSpark.stopMotor();
  }

  @Override
  public void setPosition(double position) {
    elevatorController.setTrackedReference(position, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    new Thread(
            () -> {
              tryUntilOk(
                  leaderSpark,
                  5,
                  () ->
                      leaderSpark.configure(
                          config.idleMode(brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast),
                          ResetMode.kResetSafeParameters,
                          PersistMode.kPersistParameters));
              tryUntilOk(
                  followerSpark,
                  5,
                  () ->
                      followerSpark.configure(
                          config.idleMode(brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast),
                          ResetMode.kResetSafeParameters,
                          PersistMode.kPersistParameters));
            })
        .start();
  }
}
