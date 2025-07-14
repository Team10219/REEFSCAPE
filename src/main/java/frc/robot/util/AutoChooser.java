// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Add your docs here. */
public class AutoChooser {
  private final LoggedDashboardChooser<Command> chooser;

  public AutoChooser(Drive drive) {
    chooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    chooser.addDefaultOption("Do Nothing", Commands.none());

    if (Constants.tuningMode == true) {
      chooser.addOption(
          "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
      chooser.addOption(
          "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
      chooser.addOption(
          "Drive SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      chooser.addOption(
          "Drive SysId (Quasistatic Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      chooser.addOption(
          "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      chooser.addOption(
          "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  public Command getSelected() {
    return chooser.get();
  }
}
