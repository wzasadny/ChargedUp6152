// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePieceAction;
import frc.robot.subsystems.Intake;

public class FloorIntakeRun extends CommandBase {
  private final Intake m_Intake;
  private final boolean m_On;
  private final GamePieceAction m_Action;

  /** Creates a new LowerIntakeRun. */
  public FloorIntakeRun(boolean on, GamePieceAction action, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_On = on;
    m_Action = action;
    m_Intake = intake;

    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( ) {
    m_Intake.runIntake(m_On, m_Action);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.runIntake(false, null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
