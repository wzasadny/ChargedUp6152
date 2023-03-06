// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TimedAuto extends SequentialCommandGroup {
  /** Creates a new AdvanceAuto. */
  public TimedAuto(DriveTrain drivetrain, Arm arm, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new autoArmIntakeRun(arm, 0.15, GamePiece.CONE, GamePieceAction.GRAB),
      new ParallelCommandGroup (new autoMoveArm(arm, ArmDirection.UP, 2.0), new autoArmIntakeRun(arm, 2.0, GamePiece.CONE, GamePieceAction.HOLD)),
      new ParallelCommandGroup(new autoArmIntakeRun(arm, 0.5, GamePiece.CONE, GamePieceAction.EJECT), new autoMoveIntake(intake, 0.5)),
      new autoMoveArm(arm, ArmDirection.DOWN,  2.0),
      new DriveToDistance(4.0, drivetrain, Direction.BACKWARD)
      );
  }
}
