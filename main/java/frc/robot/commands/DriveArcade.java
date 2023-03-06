// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveArcade extends CommandBase {
  private final DriveTrain m_DriveTrain;
  private final DoubleSupplier m_MoveSpeed;
  private final DoubleSupplier m_RotateSpeed;

  /** Creates a new DriveArcade. */
  public DriveArcade(DoubleSupplier moveSpeed, DoubleSupplier rotateSpeed, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveTrain = driveTrain;
    m_MoveSpeed = moveSpeed;
    m_RotateSpeed = rotateSpeed;

    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveTrain.arcadeDrive(-0.8 * m_MoveSpeed.getAsDouble(), 0.5 * m_RotateSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
