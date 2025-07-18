// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.canID.intakeID.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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
 * 6328 does a cool level of abstraction with their rollers where they dont specify a canID so they
 * can use the same code over again, but our robot is so simple thats so unnecessary, they also use
 * SparkBase and then check if its a sparkflex or sparkmax, but we only have sparkmax. The goal here
 * is to just simplify the intake with the ability to specify the velocity, the voltage, etc.
 * Specifically because the code i did during season was mad ugly. Maybe i should also code a
 * talonFX one incase we change to krakens
 */
public class IntakeIOSpark implements IntakeIO {
  private final SparkMax leftSpark;
  private final RelativeEncoder leftEncoder;
  private final TrackedController leftController;

  private final SparkMax rightSpark;
  private final RelativeEncoder rightEncoder;
  private final TrackedController rightController;

  private final SparkMaxConfig config;

  private double maxAcceleration = 10000;
  private double maxVelocity = 4000;
  private double Kv = 473;
  private int currentLimit = 50;
  private int freeLimit = 40;
  private boolean brakeModeEnabled = true;

  public IntakeIOSpark() {
    leftSpark = new SparkMax(left, MotorType.kBrushless);
    leftEncoder = leftSpark.getEncoder();
    leftController = new TrackedController(leftSpark.getClosedLoopController(), leftEncoder);

    rightSpark = new SparkMax(right, MotorType.kBrushless);
    rightEncoder = rightSpark.getEncoder();
    rightController = new TrackedController(rightSpark.getClosedLoopController(), rightEncoder);

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
        .p(0)
        .i(0)
        .d(0)
        .p(0, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .maxMotion
        .maxAcceleration(maxAcceleration)
        .maxVelocity(maxVelocity);

    tryUntilOk(
        leftSpark,
        5,
        () ->
            leftSpark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        rightSpark,
        5,
        () ->
            rightSpark.configure(
                config.inverted(true),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    sparkStickyFault = false;

    inputs.leftConnected = !sparkStickyFault;
    inputs.leftPositionRads =
        ifOkOrDefault(leftSpark, leftEncoder::getPosition, inputs.leftPositionRads);
    inputs.leftVelocityRadPerSec =
        ifOkOrDefault(leftSpark, leftEncoder::getVelocity, inputs.leftVelocityRadPerSec);
    inputs.leftAppliedVolts =
        ifOkOrDefault(
            leftSpark,
            new DoubleSupplier[] {leftSpark::getBusVoltage, leftSpark::getAppliedOutput},
            x -> x[0] * x[1],
            inputs.leftAppliedVolts);
    inputs.leftCurrentAmps =
        ifOkOrDefault(leftSpark, leftSpark::getOutputCurrent, inputs.leftCurrentAmps);
    inputs.leftTempCelsius =
        ifOkOrDefault(leftSpark, leftSpark::getMotorTemperature, inputs.leftTempCelsius);
    inputs.leftControlType = leftController.getControlType();

    inputs.rightConnected = !sparkStickyFault;
    inputs.rightPositionRads =
        ifOkOrDefault(rightSpark, rightEncoder::getPosition, inputs.rightPositionRads);
    inputs.rightVelocityRadPerSec =
        ifOkOrDefault(rightSpark, rightEncoder::getVelocity, inputs.rightVelocityRadPerSec);
    inputs.rightAppliedVolts =
        ifOkOrDefault(
            rightSpark,
            new DoubleSupplier[] {rightSpark::getBusVoltage, rightSpark::getAppliedOutput},
            x -> x[0] * x[1],
            inputs.rightAppliedVolts);
    inputs.rightCurrentAmps =
        ifOkOrDefault(rightSpark, rightSpark::getOutputCurrent, inputs.rightCurrentAmps);
    inputs.rightTempCelsius =
        ifOkOrDefault(rightSpark, rightSpark::getMotorTemperature, inputs.rightTempCelsius);
    inputs.rightControlType = rightController.getControlType();
  }

  @Override
  public void runOpenLoop(double output) {
    leftSpark.set(output);
    rightSpark.set(output);
  }

  @Override
  public void runVolts(double volts) {
    leftSpark.setVoltage(volts);
    rightSpark.setVoltage(volts);
  }

  @Override
  public void runSeperateVolts(double leftVolts, double rightVolts) {
    leftSpark.setVoltage(leftVolts);
    rightSpark.setVoltage(rightVolts);
  }

  @Override
  public void stop() {
    leftSpark.stopMotor();
    rightSpark.stopMotor();
  }

  @Override
  public void runVelocity(double velocity) {
    leftController.setTrackedReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    rightController.setTrackedReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void runVelocityMAXMotion(double velocity) {
    leftController.setTrackedReference(
        velocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
    rightController.setTrackedReference(
        velocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
  }

  @Override
  public void setPIDV(double kP, double kI, double kD, double vF) {
    config.closedLoop.p(kP).i(kI).d(kD).velocityFF(vF);

    tryUntilOk(
        leftSpark,
        5,
        () ->
            leftSpark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        rightSpark,
        5,
        () ->
            rightSpark.configure(
                config.inverted(true),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    new Thread(
            () -> {
              tryUntilOk(
                  leftSpark,
                  5,
                  () ->
                      leftSpark.configure(
                          config.idleMode(brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast),
                          ResetMode.kResetSafeParameters,
                          PersistMode.kPersistParameters));
              tryUntilOk(
                  rightSpark,
                  5,
                  () ->
                      rightSpark.configure(
                          config.idleMode(brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast),
                          ResetMode.kResetSafeParameters,
                          PersistMode.kPersistParameters));
            })
        .start();
  }
}
