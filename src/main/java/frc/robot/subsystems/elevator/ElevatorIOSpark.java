// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Robot;
import java.util.List;

/** Add your docs here. */
public class ElevatorIOSpark implements ElevatorIO {
  final double elevatorGearing = 9.0;

  final SparkMax leader, follower;
  final SparkClosedLoopController elevatorClosedLoopController;
  final RelativeEncoder leaderEncoder;
  final List<SparkMax> motors;
  private final SparkMaxConfig config;

  final double position;
  final double velocity;
  double target;

  private final Debouncer leaderConnectedDebounce = new Debouncer(0.5);
  private final Debouncer followerConnectedDebounce = new Debouncer(0.5);

  private SparkMaxSim leaderSim, followerSim;
  private List<SparkMaxSim> motorSims;
  private ElevatorSim elevatorSim;

  public ElevatorIOSpark() {
    leader = new SparkMax(ElevatorCAN.leaderCAN, MotorType.kBrushless);
    follower = new SparkMax(ElevatorCAN.followerCAN, MotorType.kBrushless);
    elevatorClosedLoopController = leader.getClosedLoopController();
    motors = List.of(leader, follower);
    leaderEncoder = leader.getEncoder();

    position = leaderEncoder.getPosition();
    velocity = leaderEncoder.getVelocity();

    target = 0.0;

    config = new SparkMaxConfig();
    final boolean inverted = false;

    config.inverted(inverted);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.065)
        .i(0.000007)
        .d(0.24)
        .outputRange(-1, 1)
        .maxMotion
        .maxVelocity(4000)
        .maxAcceleration(6000)
        .allowedClosedLoopError(0.45);
    config.smartCurrentLimit(80).voltageCompensation(12);

    tryUntilOk(
        leader,
        5,
        () ->
            leader.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        follower,
        5,
        () ->
            follower.configure(
                config.follow(ElevatorCAN.leaderCAN, true),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    if (Robot.isReal()) return;

    leaderSim = new SparkMaxSim(leader, DCMotor.getNEO(1));
    followerSim = new SparkMaxSim(follower, DCMotor.getNEO(1));

    motorSims = List.of(leaderSim, followerSim);

    elevatorSim =
        new ElevatorSim(DCMotor.getNEO(2), elevatorGearing, 5.4, 0.11176, 0, 1.01, true, 0);
    elevatorSim.update(0);
  }

  @Override
  public void setPower(double power) {
    setVoltage(power * 12);
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setPosition(double currentTarget) {
    target = currentTarget;
    elevatorClosedLoopController.setReference(currentTarget, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void update(ElevatorIOInputs inputs) {
    sparkStickyFault = false;
    inputs.leaderConnected = leaderConnectedDebounce.calculate(!sparkStickyFault);
    inputs.followerConnected = followerConnectedDebounce.calculate(!sparkStickyFault);
    inputs.leaderTemp = leader.getMotorTemperature();
    inputs.followerTemp = follower.getMotorTemperature();
    inputs.position = position;
    inputs.velocity = velocity;
    inputs.target = target;
    inputs.bottomLimit = false;
  }

  @Override
  public void simulationPeriodic() {
    elevatorSim.setInputVoltage(leader.getAppliedOutput() * 12);
    elevatorSim.update(0.02);

    motorSims.forEach(
        motorSim -> {
          motorSim.setPosition(elevatorSim.getPositionMeters() * elevatorGearing / 2.0);
          motorSim.setVelocity(elevatorSim.getVelocityMetersPerSecond() * elevatorGearing / 2.0);
          motorSim.setMotorCurrent(elevatorSim.getCurrentDrawAmps() / 2.0);
        });
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void zero() {
    leaderEncoder.setPosition(0);
  }

  @Override
  public void brakeMode(boolean brakeEnabled) {
    motors.forEach(
        motor ->
            tryUntilOk(
                motor,
                5,
                () ->
                    motor.configure(
                        config.idleMode(brakeEnabled ? IdleMode.kBrake : IdleMode.kCoast),
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters)));
  }
}
