// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.GamePieceAction;
import frc.robot.subsystems.Arm;

public class ArmIntakeRun extends CommandBase {
  private final Arm m_Arm;
  private final boolean m_On;
  private final GamePieceAction m_Action;
  private final GamePiece m_Piece;

  /** Creates a new UpperIntakeRun. */
  public ArmIntakeRun(boolean on, GamePieceAction action, GamePiece piece, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    m_On = on;
    m_Action = action;
    m_Arm = arm;
    m_Piece = piece;

    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.runArmIntake(m_On, m_Piece, m_Action);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.runArmIntake(false, null, null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
