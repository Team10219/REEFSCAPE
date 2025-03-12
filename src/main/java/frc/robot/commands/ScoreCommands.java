// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorConstants.Setpoint;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

/** Add your docs here. */
public class ScoreCommands {
  public Command ScoreL1(ElevatorSubsystem m_elevator, IntakeSubsystem m_intake) {
    return m_elevator.setSetpointCommand(Setpoint.kLevel1).andThen(m_intake.Outtake());
  }

  public Command ScoreL2(ElevatorSubsystem m_elevator, IntakeSubsystem m_intake) {
    return m_elevator.setSetpointCommand(Setpoint.kLevel2).andThen(m_intake.Outtake());
  }

  public Command ScoreL3(ElevatorSubsystem m_elevator, IntakeSubsystem m_intake) {
    return m_elevator.setSetpointCommand(Setpoint.kLevel3).andThen(m_intake.Outtake());
  }

  public Command ScoreL4(ElevatorSubsystem m_elevator, IntakeSubsystem m_intake) {
    return m_elevator.setSetpointCommand(Setpoint.kLevel4).andThen(m_intake.Outtake());
  }
}
