// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import java.util.List;

/** Add your docs here. */
// the goal of this is to have the ability to control the motors individualy
public class IntakeIOSpark implements IntakeIO {
  private final double gearing = 5 / 1;
  private final double wheelDiameterMeters = Units.inchesToMeters(2);
  private final double wheelCircumference = Math.PI * wheelDiameterMeters;

  private final double MOI = 0.2;
  private final double Kv = 473;
  private int currentLimit = 30;
  private int freeLimit = 40;
  private boolean brakeModeEnabled = true;

  private final SparkMax left, right;
  private final RelativeEncoder leftEncoder, rightEncoder;
  private final SparkClosedLoopController leftController, rightController;
  private final List<SparkMax> motors;
  private final SparkMaxConfig config;

  private final Debouncer leftConnectedDebounce = new Debouncer(0.5);
  private final Debouncer rightConnectedDebounce = new Debouncer(0.5);

  private SparkMaxSim leftSim, rightSim;
  private List<SparkMaxSim> motorSims;
  private FlywheelSim leftIntakeSim, rightIntakeSim;
  private List<FlywheelSim> intakeSims;

  public IntakeIOSpark() {
    left = new SparkMax(4, MotorType.kBrushless);
    right = new SparkMax(5, MotorType.kBrushless);
    leftController = left.getClosedLoopController();
    rightController = right.getClosedLoopController();
    motors = List.of(left, right);
    leftEncoder = left.getEncoder();
    rightEncoder = right.getEncoder();

    config = new SparkMaxConfig();

    
    config
        .idleMode(brakeModeEnabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(currentLimit, freeLimit)
        .voltageCompensation(12.0)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1)
        .velocityFF(1/Kv)
        .maxMotion
        .maxAcceleration(4000)
        .maxVelocity(4000);

    tryUntilOk(
        left,
        5,
        () ->
            left.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        right,
        5,
        () ->
            right.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(left, 5, () -> leftEncoder.setPosition(0));
    tryUntilOk(right, 5, () -> rightEncoder.setPosition(0));

    if (RobotBase.isReal()) return;

    leftSim = new SparkMaxSim(left, DCMotor.getNEO(1));
    rightSim = new SparkMaxSim(right, DCMotor.getNEO(1));

    leftIntakeSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 3 * MOI, gearing),
            DCMotor.getNEO(1));
    rightIntakeSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 3 * MOI, gearing),
            DCMotor.getNEO(1));
  }

  // While I would love to group these together, they are not mechanically linked at all and they
  // need to run seperately so they have to be completely seperate pieces of code
  @Override
  public void simulationPeriodic() {
    leftIntakeSim.setInputVoltage(leftSim.getAppliedOutput() * 12);
    rightIntakeSim.setInputVoltage(rightSim.getAppliedOutput() * 12);

    leftIntakeSim.update(Constants.loopPeriodSecs);
    rightIntakeSim.update(Constants.loopPeriodSecs);

    leftSim.setVelocity(leftIntakeSim.getAngularVelocityRPM());
    leftSim.setMotorCurrent(leftIntakeSim.getCurrentDrawAmps());
    rightSim.setVelocity(rightIntakeSim.getAngularVelocityRPM());
    rightSim.setMotorCurrent(rightIntakeSim.getCurrentDrawAmps());
  }


  @Override
  public void setPower(double leftPower, double rightPower) {
    setVoltage(leftPower * 12, rightPower * 12);
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(-rightVolts);
  }

  @Override
  public void setVelocity(
      double leftVelTarget,
      SparkBase.ControlType leftControlType,
      double rightVelTarget,
      SparkBase.ControlType rightControlType) {
    leftController.setReference(leftVelTarget, leftControlType);
    rightController.setReference(rightVelTarget, rightControlType);
  }

  @Override
  public void stop() {
    motors.forEach(motor -> motor.stopMotor());
  }

  @Override
  public void update(IntakeIOInputs inputs) {
    sparkStickyFault = false;
    inputs.leftConnected = leftConnectedDebounce.calculate(!sparkStickyFault);
    inputs.rightConnected = rightConnectedDebounce.calculate(!sparkStickyFault);
    inputs.leftTemp = left.getMotorTemperature();
    inputs.rightTemp = right.getMotorTemperature();
    inputs.leftVelocityRPM = (leftEncoder.getVelocity() * wheelCircumference) / (gearing);
    inputs.rightVelocityRPM = (rightEncoder.getVelocity() * wheelCircumference) / (gearing);
  }

  @Override
  public void brakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    new Thread(
      () ->
      motors.forEach(
          motor ->
              tryUntilOk(
                  motor,
                  5,
                  () ->
                      motor.configure(
                          config.idleMode(brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast),
                          ResetMode.kResetSafeParameters,
                          PersistMode.kPersistParameters))))
        .start(); 
  }
}
