// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotType;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsXbox;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.vision.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;
  private Elevator elevator;

  private DriverControls driverControls;

  private SwerveDriveSimulation driveSimulation = null;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public static RobotContainer instance;

  public RobotContainer() {
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case COMPBOT:
          // Real robot, instantiate hardware IO implementations
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                  new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                  new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                  new ModuleIOTalonFXReal(TunerConstants.BackRight),
                  (pose) -> {});
          this.vision =
              new Vision(
                  drive, new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
          elevator = new Elevator(new ElevatorIOSpark());

          break;
        case SIMBOT:
          // Sim robot, instantiate physics sim IO implementations

          driveSimulation =
              new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
          SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
          drive =
              new Drive(
                  new GyroIOSim(driveSimulation.getGyroSimulation()),
                  new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                  new ModuleIOTalonFXSim(
                      TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                  new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                  new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                  driveSimulation::setSimulationWorldPose);
          vision =
              new Vision(
                  drive,
                  new VisionIOPhotonVisionSim(
                      camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose));
          elevator = new Elevator(new ElevatorIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  (pose) -> {});
          vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
          elevator = new Elevator(new ElevatorIOSim());
          break;
      }
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureControllers();
    configureButtonBindings();
  }

  private void configureControllers() {
    driverControls = new DriverControlsXbox(0);
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, driverControls::getForward, driverControls::getStrafe, driverControls::getTurn));

    // Lock to 0° when A button is held
    driverControls
        .lockToZero()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                driverControls::getForward,
                driverControls::getStrafe,
                () -> new Rotation2d()));

    driverControls
        .reefFace()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                driverControls::getForward,
                driverControls::getStrafe,
                () -> new Rotation2d().fromDegrees(60)));

    // Switch to X pattern when X button is pressed
    driverControls.xWheels().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro / odometry
    final Runnable resetGyro =
        Constants.getRobot() == RobotType.SIMBOT
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
            : () ->
                drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    driverControls
        .resetFieldCentric()
        .onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    driverControls.autoAlign(false).onTrue(new AutoAlignCommand(false, drive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.getRobot() != RobotType.SIMBOT) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.getRobot() != RobotType.SIMBOT) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
