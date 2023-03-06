// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmDirection;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.GamePieceAction;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  final CANSparkMax arm_Motor = new CANSparkMax(Constants.ARM_MOTOR, MotorType.kBrushless);
  final CANSparkMax arm_Scoring_Motor = new CANSparkMax(Constants.ARM_SCORING_MOTOR, MotorType.kBrushless);

  public Arm() {
    arm_Motor.getEncoder().setPosition(0);
    arm_Motor.setInverted(true);
    arm_Motor.setIdleMode(IdleMode.kBrake);
    arm_Motor.setSmartCurrentLimit(Constants.ARM_CURRENT_LIMIT_A);
    arm_Scoring_Motor.setInverted(false);
    arm_Scoring_Motor.setIdleMode(IdleMode.kBrake);
  }

  //Gets encoders distance
  public double getArmDistance() {
    double armPosition = arm_Motor.getEncoder().getPosition();
        
    return (Math.PI * 0.1524 * armPosition / 4096.0); // FIX THIS

    /* A better implimentation for the arm would be to include a PID control to ensure that the arm reaches the correct height everytime */
  }

  //Moves arm in specified direction
  public void moveArm (boolean pressed, ArmDirection direction) {
    if (pressed) {
      if (direction == ArmDirection.UP){
        arm_Motor.set(Constants.ARM_OUTPUT_POWER);
      }
      else {
       arm_Motor.set(-Constants.ARM_OUTPUT_POWER);
      }
    }
    else{
      arm_Motor.set(0);
    }
  }

  public void resetArmEncoder() {
    arm_Motor.getEncoder().setPosition(0);
  }

  public void runArmIntake(boolean pressed, GamePiece piece, GamePieceAction action) {
    // Grabs or ejects the game piece when pressed
    double intakePower;
    int intakeAmps;

    if (pressed) {
      if ((action == GamePieceAction.GRAB && piece == GamePiece.CUBE) || (action == GamePieceAction.EJECT && piece == GamePiece.CONE)) {
        intakePower = Constants.ARM_INTAKE_OUTPUT_POWER;
        intakeAmps = Constants.ARM_INTAKE_CURRENT_LIMIT;
      }
      else if ((action == GamePieceAction.GRAB && piece == GamePiece.CONE) || (action == GamePieceAction.EJECT && piece == GamePiece.CUBE)) {
        intakePower = -Constants.ARM_INTAKE_OUTPUT_POWER;
        intakeAmps = Constants.ARM_INTAKE_CURRENT_LIMIT;
      }
      else if (action == GamePieceAction.HOLD && piece == GamePiece.CUBE) {
        intakePower = Constants.ARM_INTAKE_HOLD_POWER;
        intakeAmps = Constants.ARM_INTAKE_HOLD_CURRENT_LIMIT_A;
      }
      else if (action == GamePieceAction.HOLD && piece == GamePiece.CONE) {
        intakePower = -Constants.ARM_INTAKE_HOLD_POWER;
        intakeAmps = Constants.ARM_INTAKE_HOLD_CURRENT_LIMIT_A;
      }
      else {
       intakePower = 0.0;
       intakeAmps = 0;
      }
    }
    else {
      intakePower = 0.0;
      intakeAmps = 0;
    }
  
    arm_Scoring_Motor.set(intakePower);
    arm_Scoring_Motor.setSmartCurrentLimit(intakeAmps);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getArmDistance();
  }

}