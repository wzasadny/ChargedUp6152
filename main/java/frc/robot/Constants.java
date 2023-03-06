// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  //DriveTrain Motor Channels
  public static int FRONT_RIGHT = 1;
  public static int FRONT_LEFT = 2;
  public static int BACK_RIGHT = 3;
  public static int BACK_LEFT = 4;

  //Subsystem Motor Channels
  public static int ARM_MOTOR = 5;
  public static int ARM_SCORING_MOTOR = 6;
  public static int INTAKE_MOTOR  = 7;
  public static int INTAKE_SCORING_MOTOR = 8;

  //Controller Constants Buttons
  public static int GREEN_BUTTON = 1;
  public static int RED_BUTTON = 2;
  public static int BLUE_BUTTON = 3;
  public static int YELLOW_BUTTON = 4;
  public static int TOP_LEFT_BUTTON = 5;
  public static int TOP_RIGHT_BUTTON = 6;
  public static int BACK_BUTTON  = 7;
  public static int START_BUTTON  = 8;
  public static int LEFT_JOYSTICK_BUTTON = 9;
  public static int RIGHT_JOYSTICK_BUTTON = 10;

  // Current Limits and Power Settings for Arm and Arm-Intake
  public static int ARM_CURRENT_LIMIT_A = 20;
  public static double ARM_OUTPUT_POWER = 0.4;

  public static int ARM_INTAKE_CURRENT_LIMIT = 25;
  public static int ARM_INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  public static double ARM_INTAKE_OUTPUT_POWER = 1.0;
  public static double ARM_INTAKE_HOLD_POWER = 0.07;

  // Current Limits and Power Settings for Intake and Grabber
  public static int INTAKE_CURRENT_LIMIT_A = 25;
  public static double INTAKE_LOWER_POWER = -0.4;
  public static double INTAKE_RAISE_POWER = 0.6;
  public static int INTAKE_GRABBER_HOLD_CURRENT_LIMIT_A = 5;

  public static double INTAKE_EJECT_POWER = -1.0;
  public static double INTAKE_GRAB_POWER = 1.0;
  public static double INTAKE_HOLD_POWER = 0.07;
  
  //OI Constants
  public static class OperatorConstants {
    public static final int kDriverONEControllerPort = 0;
    public static final int kDriverTWOControllerPort = 1;
    public static final int kDriverControllerRightDirectionalPad = 1;
    public static final int kDriverControllerLeftDirectionalPad = 4;
  }

  public enum ArmDirection {
    UP, 
    DOWN
  }
  
  public enum IntakeDirection {
    RAISE,
    LOWER
  }

  public enum GamePieceAction {
    GRAB,
    EJECT,
    HOLD
  }

  public enum GamePiece {
    CONE,
    CUBE
  }

  public enum Direction {
    FORWARD,
    BACKWARD
  }
}
