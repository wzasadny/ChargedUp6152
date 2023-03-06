// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Direction;
import frc.robot.subsystems.DriveTrain;

public class DriveToDistance extends CommandBase {
  private final DriveTrain m_driveTrain;
  private final double m_distance;
  private  double m_time;
  Direction m_direction;

  /** Creates a new DriveToDistance. */
  public DriveToDistance(double distance, DriveTrain drivetrain, Direction direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = drivetrain;
    m_distance = distance;
    m_direction = direction;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetEncoders();
    m_time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_direction == Direction.FORWARD) {
      m_driveTrain.arcadeDrive(0.5, 0.0);
    }
    else if (m_direction == Direction.BACKWARD) {
      m_driveTrain.arcadeDrive(-0.5, 0.0);
    }
    else {
      m_driveTrain.arcadeDrive(0.0, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_driveTrain.getDistance() >= m_distance - 0.1) {
      return true;
    }
    else if (Timer.getFPGATimestamp() - m_time >= 4.0) {
      return true;
    }
    else {
      return false;
    } 
  }
}
