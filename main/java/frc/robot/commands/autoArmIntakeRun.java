// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.GamePieceAction;
import frc.robot.subsystems.Arm;

public class autoArmIntakeRun extends CommandBase {
  /** Creates a new autoArmIntakeRun. */
  Arm m_arm;
  double m_startTime, m_time;
  GamePiece m_piece;
  GamePieceAction m_action;

  public autoArmIntakeRun(Arm arm, double time, GamePiece piece, GamePieceAction action) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    m_time = time;
    m_piece = piece;
    m_action = action;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.runArmIntake(true, m_piece, m_action);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_arm.runArmIntake(false, null, null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - m_startTime >= m_time);
  }
}
