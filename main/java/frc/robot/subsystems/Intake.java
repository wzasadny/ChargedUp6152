// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GamePieceAction;
import frc.robot.Constants.IntakeDirection;

public class Intake extends SubsystemBase {
  /** Creates a new Elevator. */
  final CANSparkMax intake_Motor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushed);
  final CANSparkMax intake_Scoring_Motor = new CANSparkMax(Constants.INTAKE_SCORING_MOTOR, MotorType.kBrushed);

  public Intake() {
    intake_Motor.setSmartCurrentLimit(Constants.INTAKE_CURRENT_LIMIT_A);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveIntake(boolean pressed, IntakeDirection direction) {
    if (pressed) {
      if (direction == IntakeDirection.LOWER) {
        intake_Motor.set(-Constants.INTAKE_LOWER_POWER);
      }
      else {
        intake_Motor.set(Constants.INTAKE_RAISE_POWER);
      }
    }
    else{
      intake_Motor.set(0);
    }
   }

  public void runIntake(boolean pressed, GamePieceAction action) {
    //Runs Intake when button is pressed
    if (pressed) {
      if (action == GamePieceAction.GRAB) {
        intake_Scoring_Motor.set(Constants.INTAKE_GRAB_POWER);
      }
      else if (action == GamePieceAction.EJECT) {
        intake_Scoring_Motor.set(Constants.INTAKE_EJECT_POWER);
      }
      else if (action == GamePieceAction.HOLD) {
        intake_Scoring_Motor.set(Constants.INTAKE_HOLD_POWER);
      }
    }
    else {
      intake_Scoring_Motor.set(0);
    }
  } 
}