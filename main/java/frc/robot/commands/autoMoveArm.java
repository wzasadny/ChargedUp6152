// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmDirection;
import frc.robot.subsystems.Arm;

public class autoMoveArm extends CommandBase {
  /** Creates a new autoArmUp. */
  double m_startTime, m_time;
  Arm m_arm;
  ArmDirection m_direction;

  public autoMoveArm(Arm arm, ArmDirection direction, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    m_direction = direction;
    m_time = time;
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
    m_arm.moveArm(true, m_direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.moveArm(false, null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - m_startTime) >= m_time;
  }
}
