// Created by 10219 Bathtub Chickens
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

/**
 * Maintains the same functionality as SparkClosedLoopController, however grants the ability to
 * track the current ControlType for logging and other purposes
 */
public class TrackedController {
  private final SparkClosedLoopController controller;
  private ControlType controlType = null;
  private double IAccum = 0.0;

  public TrackedController(SparkClosedLoopController controller) {
    this.controller = controller;
  }

  public void setTrackedReference(double value, ControlType type) {
    controller.setReference(value, type);
    this.controlType = type;
  }

  public void setTrackedReference(double value, ControlType type, ClosedLoopSlot slot) {
    controller.setReference(value, type, slot);
    this.controlType = type;
  }

  public void setTrackedReference(
      double value, ControlType type, ClosedLoopSlot slot, double arbFeedforward) {
    controller.setReference(value, type, slot, arbFeedforward);
    this.controlType = type;
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
}
