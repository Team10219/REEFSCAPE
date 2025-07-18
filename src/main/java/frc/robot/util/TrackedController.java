// Created by 10219 Bathtub Chickens
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import java.util.function.BooleanSupplier;

/**
 * Maintains the same functionality as SparkClosedLoopController, however grants the ability to
 * track the current ControlType for logging and other purposes
 */
public class TrackedController {
  private final SparkClosedLoopController controller;
  private final RelativeEncoder encoder;
  private ControlType controlType = null;
  private double IAccum = 0.0;
  private double setpoint = 0.0;

  public TrackedController(SparkClosedLoopController controller, RelativeEncoder encoder) {
    this.controller = controller;
    this.encoder = encoder;
  }

  public void setTrackedReference(double setpoint, ControlType type) {
    controller.setReference(setpoint, type);
    this.setpoint = setpoint;
    this.controlType = type;
  }

  public void setTrackedReference(double setpoint, ControlType type, ClosedLoopSlot slot) {
    controller.setReference(setpoint, type, slot);
    this.setpoint = setpoint;
    this.controlType = type;
  }

  public void setTrackedReference(
      double setpoint, ControlType type, ClosedLoopSlot slot, double arbFeedforward) {
    controller.setReference(setpoint, type, slot, arbFeedforward);
    this.setpoint = setpoint;
    this.controlType = type;
  }

  public void setTrackedReference(
      double setpoint,
      ControlType type,
      ClosedLoopSlot slot,
      double arbFeedforward,
      ArbFFUnits arbFFUnits) {
    controller.setReference(setpoint, type, slot, arbFeedforward, arbFFUnits);
    this.setpoint = setpoint;
    this.controlType = type;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public ControlType getControlType() {
    return controlType;
  }

  public boolean hasControlType() {
    return controlType != null;
  }

  public REVLibError setIAccum(double IAccum) {
    this.IAccum = IAccum;
    return controller.setIAccum(IAccum);
  }

  public double getIAccum() {
    return IAccum;
  }

  public SetpointChecker atSetpoint() {
    return new SetpointChecker();
  }

  public class SetpointChecker implements BooleanSupplier {

    @Override
    public boolean getAsBoolean() {
      return withinTolerance(0.0);
    }

    @SuppressWarnings("removal")
    public boolean withinTolerance(double tolerance) {
      if (controlType == null) return false;

      try {
        switch (controlType) {
          case kMAXMotionPositionControl:
          case kPosition:
          case kSmartMotion:
            return Math.abs(encoder.getPosition() - setpoint) <= tolerance;
          case kMAXMotionVelocityControl:
          case kVelocity:
          case kSmartVelocity:
            return Math.abs(encoder.getVelocity() - setpoint) <= tolerance;
          case kVoltage:
          case kCurrent:
          case kDutyCycle:
            return true;
            /* Try and find a better way but since these are open loop modes idk if there is*/
          default:
            return false;
        }
      } catch (Exception e) {
        return false;
      }
    }
  }
}
