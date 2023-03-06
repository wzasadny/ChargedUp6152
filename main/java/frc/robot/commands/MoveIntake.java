// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeDirection;
import frc.robot.subsystems.Intake;

public class MoveIntake extends CommandBase {
  /** Creates a new MoveElevator. */
  private final Intake m_Intake;
  private final boolean m_On;
  private final IntakeDirection m_Direction;

  public MoveIntake(boolean on, IntakeDirection direction, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Intake = intake;
    m_On = on;
    m_Direction = direction;

    addRequirements(m_Intake);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_Intake.moveIntake(m_On, m_Direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.moveIntake(false, null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
