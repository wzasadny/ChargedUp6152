// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final DriveTrain m_DriveTrain = new DriveTrain();
  private final Intake m_Intake = new Intake();
  private final Arm m_Arm = new Arm();
  
  private final XboxController m_DriverONE = new XboxController(OperatorConstants.kDriverONEControllerPort);
  private final XboxController m_DriverTWO = new XboxController(OperatorConstants.kDriverTWOControllerPort);

  //private final Command m_ConeBasicAuto = new ConeBasicAuto();
  //private final Command m_CubeBasicAuto = new CubeBasicAuto();
  //private final Command m_AdvanceAuto = new AdvanceAuto();
  //private final Command m_DriveToDistance = new DriveToDistance(3.00, m_DriveTrain);
  //private final Command m_autoMoveArmUp = new autoMoveArm(m_Arm, ArmDirection.Up, 2.00);

  // Used to send the various options for autonomous to the SmartDashboard
  SendableChooser<PathPlannerTrajectory> m_paths = new SendableChooser<>();
  SendableChooser<GamePiece> m_gamepiece = new SendableChooser<>();
  SendableChooser<String> m_auto = new SendableChooser<>();

  // ** Path planner auto code **
  PathPlannerTrajectory m_blueLeftPath = PathPlanner.loadPath("LeftBlue", new PathConstraints(3,2));
  PathPlannerTrajectory m_blueRightPath = PathPlanner.loadPath("RightBlue", new PathConstraints(3,2));
  PathPlannerTrajectory m_blueCenterPath = PathPlanner.loadPath("MiddleBlue", new PathConstraints(3,2));
  PathPlannerTrajectory m_RedLeftPath = PathPlanner.loadPath("LeftRed", new PathConstraints(3,2));
  PathPlannerTrajectory m_RedRightPath = PathPlanner.loadPath("RightRed", new PathConstraints(3,2));
  PathPlannerTrajectory m_RedCenterPath = PathPlanner.loadPath("MiddleRed", new PathConstraints(3,2));

  GamePiece m_CONE = Constants.GamePiece.CONE;
  GamePiece m_CUBE = Constants.GamePiece.CUBE;

  String m_timedauto = "SIMPLE";
  String m_advanced = "ADVANCED TRAJECTORY";

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Creates the various autonomous options to choose from on the SMART Dashboard
    // m_chooser.setDefaultOption("Drive to Distance", m_DriveToDistance);
    m_paths.setDefaultOption("Blue Left Path", m_blueLeftPath);
    m_paths.addOption("Blue Right Path", m_blueRightPath);
    m_paths.addOption("Blue Center Path", m_blueCenterPath);
    m_paths.addOption("Red Left Path", m_RedLeftPath);
    m_paths.addOption("Red Right Path", m_RedRightPath);
    m_paths.addOption("Red Center Path", m_RedCenterPath);
    
    // Creates a chooser for the starting game piece to choose from on the SMART Dashboard
    m_gamepiece.setDefaultOption("CONE", m_CONE);
    m_gamepiece.addOption("CUBE", m_CUBE);

    m_auto.setDefaultOption("SIMPLE", m_timedauto);
    m_auto.addOption("ADVANCED TRAJECTORY", m_advanced);

    //SmartDashboard.putData("Autonomous Options: ", m_chooser);
    SmartDashboard.putData("Autonomus Paths: ", m_paths);
    SmartDashboard.putData("Starting Game Piece: ", m_gamepiece);
    SmartDashboard.putData("Auto Type: ", m_auto);


    //Moves robot based on joystick's pos.
    m_DriveTrain.setDefaultCommand(new DriveArcade(() -> m_DriverONE.getRawAxis(OperatorConstants.kDriverControllerRightDirectionalPad), 
                                                   () -> m_DriverONE.getRawAxis(OperatorConstants.kDriverControllerLeftDirectionalPad), 
                                                   m_DriveTrain));

    configureBindings();
  }

  public Command loadPPtoRamsete(PathPlannerTrajectory trajectory) {
    
    PathPlannerTrajectory m_trajectory = trajectory;

    return 
      new PPRamseteCommand(
        m_trajectory, 
        m_DriveTrain::getPose, 
        new RamseteController(2.0, 0.7),
        m_DriveTrain.getFeedforward(),
        m_DriveTrain.getKinematics(), 
        m_DriveTrain::getSpeeds, 
        m_DriveTrain.getLeftController(), 
        m_DriveTrain.getRightController(), 
        m_DriveTrain::setOutput,
        false,
        m_DriveTrain
        );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Configure the trigger bindings for Driver ONE
    JoystickButton driverONE_greenButton = new JoystickButton(m_DriverONE, Constants.GREEN_BUTTON);
    JoystickButton driverONE_redButton = new JoystickButton(m_DriverONE, Constants.RED_BUTTON);
    JoystickButton driverONE_blueButton = new JoystickButton(m_DriverONE, Constants.BLUE_BUTTON);
    JoystickButton driverONE_yellowButton = new JoystickButton(m_DriverONE, Constants.YELLOW_BUTTON);
    JoystickButton driverONE_topLeftButton = new JoystickButton(m_DriverONE, Constants.TOP_LEFT_BUTTON);
    JoystickButton driverONE_topRightButton = new JoystickButton(m_DriverONE, Constants.TOP_RIGHT_BUTTON);
    JoystickButton driverONE_startButton = new JoystickButton(m_DriverONE, Constants.START_BUTTON);

    // Configure the trigger bindings for Driver TWO
    JoystickButton driverTWO_greenButton = new JoystickButton(m_DriverTWO, Constants.GREEN_BUTTON);
    JoystickButton driverTWO_redButton = new JoystickButton(m_DriverTWO, Constants.RED_BUTTON);
    JoystickButton driverTWO_blueButton = new JoystickButton(m_DriverTWO, Constants.BLUE_BUTTON);
    //JoystickButton driverTWO_yellowButton = new JoystickButton(m_DriverTWO, Constants.YELLOW_BUTTON);
    JoystickButton driverTWO_topLeftButton = new JoystickButton(m_DriverTWO, Constants.TOP_LEFT_BUTTON);
    JoystickButton driverTWO_topRightButton = new JoystickButton(m_DriverTWO, Constants.TOP_RIGHT_BUTTON);
    //JoystickButton driverTWO_startButton = new JoystickButton(m_DriverTWO, Constants.START_BUTTON);
    
    
    // DRIVER ONE
    // GRAB a CONE or EJECT a CUBE
    driverONE_greenButton.whileTrue(new ArmIntakeRun(true, GamePieceAction.GRAB, GamePiece.CONE, m_Arm));
    driverONE_greenButton.whileTrue(new ArmIntakeRun(true, GamePieceAction.EJECT, GamePiece.CUBE, m_Arm));
    // GRAB a CUBE or EJECT a CONE
    driverONE_redButton.whileTrue(new ArmIntakeRun(true, GamePieceAction.EJECT, GamePiece.CONE, m_Arm));
    driverONE_redButton.whileTrue(new ArmIntakeRun(true, GamePieceAction.GRAB, GamePiece.CUBE, m_Arm));
    // HOLD a CONE
    driverONE_blueButton.whileTrue(new ArmIntakeRun(true, GamePieceAction.HOLD, GamePiece.CONE, m_Arm));
    // HOLD a CUBE
    driverONE_yellowButton.whileTrue(new ArmIntakeRun(true, GamePieceAction.HOLD, GamePiece.CUBE, m_Arm));
    // Moves arm forward X distance
    driverONE_topLeftButton.whileTrue(new MoveArm(true, 0.25, ArmDirection.UP, m_Arm));
    // Moves arm backward X distance
    driverONE_topRightButton.whileTrue(new MoveArm(true, 0.25, ArmDirection.DOWN, m_Arm));
    // Resets the drivetrain encoders and the drivetrain gyro
    driverONE_startButton.whileTrue(new Reset(m_DriveTrain));
    
    
    // DRIVER TWO    
    // Raises the floor intake
    driverTWO_topLeftButton.whileTrue(new MoveIntake(true, IntakeDirection.RAISE, m_Intake));
    // Lowers the floor intake
    driverTWO_topRightButton.whileTrue(new MoveIntake(true, IntakeDirection.LOWER, m_Intake));
    // GRAB CUBE
    driverTWO_greenButton.whileTrue(new FloorIntakeRun(true, GamePieceAction.GRAB, m_Intake));
    // EJECT CUBE
    driverTWO_redButton.whileTrue(new FloorIntakeRun(true, GamePieceAction.EJECT, m_Intake));
    // HOLD CUBE
    driverTWO_blueButton.whileTrue(new FloorIntakeRun(true, GamePieceAction.HOLD, m_Intake));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    Command autoCommand;
    if (m_auto.getSelected() == "SIMPLE") {
      autoCommand = new TimedAuto(m_DriveTrain, m_Arm, m_Intake);
    }
    else if (m_auto.getSelected() == "ADVANCED TRAJECTORY") {
      autoCommand = loadPPtoRamsete(m_paths.getSelected());
    }
    else {
      autoCommand = new InstantCommand();
    }
    return autoCommand;
  }
}
