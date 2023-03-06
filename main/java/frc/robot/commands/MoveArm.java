// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmDirection;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */
  private final Arm m_Arm;
  private final Double m_Distance;
  private final ArmDirection m_Direction;
  private final boolean m_On;

  public MoveArm(boolean on, Double distance, ArmDirection direction, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = arm;
    m_Distance = distance;
    m_Direction = direction;
    m_On = on;

    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Arm.resetArmEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.moveArm(m_On, m_Direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.moveArm(false, null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //When arm reaches distance stop
    if (m_Arm.getArmDistance() >= m_Distance || !m_On) {
      return true;
    }
    else {
      return false;
    }
  }
}
