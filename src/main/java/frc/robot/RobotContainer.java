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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.RobotType;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsXbox;
import frc.robot.oi.OperatorControls;
import frc.robot.oi.OperatorControlsXbox;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.vision.*;
import frc.robot.util.AutoChooser;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Elevator elevator;
  private Intake intake;
  private Vision vision;

  private DriverControls driver;
  private OperatorControls operator;

  private SwerveDriveSimulation driveSimulation = null;

  private final AutoChooser autoChooser;

  public static RobotContainer instance;

  @SuppressWarnings("unused")
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
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
          elevator = new Elevator(new ElevatorIOSpark());
          intake = new Intake(new IntakeIOSpark());
          this.vision =
              new Vision(
                  drive, new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
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
          // elevator = new Elevator(new ElevatorIOSim());
          // intake = new Intake(new IntakeIOSim(null, null, null))
          vision =
              new Vision(
                  drive,
                  new VisionIOPhotonVisionSim(
                      camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose));
          break;
      }
    }

    // Replayed robot, disable IO implementations
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              (pose) -> {});
    }
    if (vision == null) {
      vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
    }
    // Set up auto routines
    autoChooser = new AutoChooser(drive);

    configureControllers();
    configureButtonBindings();
  }

  private void configureControllers() {
    driver = new DriverControlsXbox(0);
    operator = new OperatorControlsXbox(1);
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand( // Default command, normal field-relative drive
        DriveCommands.joystickDrive(drive, driver::getForward, driver::getStrafe, driver::getTurn));

    driver // Lock to 0° when A button is held
        .lockToZero()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, driver::getForward, driver::getStrafe, () -> new Rotation2d()));

    driver
        .xWheels()
        .onTrue(
            Commands.runOnce(
                drive::stopWithX, drive)); // Switch to X pattern when X button is pressed

    final Runnable resetGyro = // Reset gyro / odometry
        Constants.getRobot() == RobotType.SIMBOT
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
            : () ->
                drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    driver.resetFieldCentric().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // All Sim
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
