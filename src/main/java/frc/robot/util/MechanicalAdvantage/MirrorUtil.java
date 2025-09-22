// Copyright (c) 2025 FRC 10219
// https://github.com/Team10219
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.MechanicalAdvantage;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.BooleanSupplier;
import lombok.Setter;

public class MirrorUtil {
  @Setter private static BooleanSupplier mirror;

  public static double apply(double y) {
    return shouldMirror() ? FieldConstants.fieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    if (!shouldMirror()) return translation;
    return new Translation2d(translation.getX(), apply(translation.getY()));
  }

  public static Pose2d apply(Pose2d pose) {
    if (!shouldMirror()) return pose;
    return new Pose2d(
        apply(pose.getTranslation()),
        new Rotation2d(pose.getRotation().getCos(), -pose.getRotation().getSin()));
  }

  public static boolean shouldMirror() {
    return mirror != null && mirror.getAsBoolean();
  }
}
