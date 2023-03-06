// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  final CANSparkMax frontLeft = new CANSparkMax(Constants.FRONT_LEFT, MotorType.kBrushless);
  final CANSparkMax frontRight = new CANSparkMax(Constants.FRONT_RIGHT, MotorType.kBrushless);
  final CANSparkMax backLeft = new CANSparkMax(Constants.BACK_LEFT, MotorType.kBrushless);
  final CANSparkMax backRight = new CANSparkMax(Constants.BACK_RIGHT, MotorType.kBrushless);

  final MotorControllerGroup m_right = new MotorControllerGroup(frontRight, backRight);
  final MotorControllerGroup m_left = new MotorControllerGroup(frontLeft, backLeft);
  
  final DifferentialDrive m_DifferentialDrive = new DifferentialDrive(m_right, m_left);
  
  AHRS m_Gyro = new AHRS(SPI.Port.kMXP);
  
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(19)); // MEASURE AND UPDATE! 2.25.23 Distance between the wheels of the robot
  DifferentialDriveOdometry odometry;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.268, 1.89, 0.243);

  Pose2d pose = new Pose2d();

  private final Field2d m_field = new Field2d();

  PIDController leftPIDController = new PIDController (9.95,0,0);
  PIDController rightPIDController = new PIDController (9.95, 0, 0);


  public DriveTrain() {
    m_right.setInverted(true);
    frontLeft.getEncoder().setPosition(0);
    frontRight.getEncoder().setPosition(0);
    resetEncoders();
    m_Gyro.calibrate();

    

    odometry = new DifferentialDriveOdometry(m_Gyro.getRotation2d(), getLeftDistance(), getRightDistance(), getPose());
  }

  public void arcadeDrive(Double moveSpeed, double rotateSpeed) {
    m_DifferentialDrive.arcadeDrive(moveSpeed, rotateSpeed);
  }

  public double getRightDistance () {
    return (Math.PI * 0.1524 * -1.0 * frontRight.getEncoder().getPosition() / 8.45);
  }

  public double getLeftDistance() {
    return (Math.PI * 0.1524 * frontLeft.getEncoder().getPosition() / 8.45);
  }

  public double getDistance() {
    return (getLeftDistance() + getRightDistance() / 2.0);
  }

  public void resetEncoders() {
    frontLeft.getEncoder().setPosition(0);
    frontRight.getEncoder().setPosition(0);
  }

  public Rotation2d getRotation() {
    return m_Gyro.getRotation2d();
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    // Returns wheel speeds in meters per second
    return new DifferentialDriveWheelSpeeds( // Changed numbers to fit our robot Updated: 2.20.23
      frontLeft.getEncoder().getVelocity() / 8.45 * Math.PI * Units.inchesToMeters(6.0) / 60,
      frontRight.getEncoder().getVelocity() / 8.45 * Math.PI * Units.inchesToMeters(6.0) / 60
    );
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public void setOutput(double leftVolts, double rightVolts) {
    frontLeft.set(leftVolts / 12.0);
    frontRight.set(rightVolts / 12.0);
  }

  public PIDController getLeftController() {
    return leftPIDController;
  }

  public PIDController getRightController() {
    return rightPIDController;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Motor Position: ", frontLeft.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Motor Position: ", -1.0 * frontRight.getEncoder().getPosition());
    SmartDashboard.putNumber("Distance: ", Math.PI * 0.1524 * (frontLeft.getEncoder().getPosition() + -1.0 * frontRight.getEncoder().getPosition()) / 17.5);
    SmartDashboard.putNumber("Angle: ", m_Gyro.getAngle());
    SmartDashboard.putData("Field", m_field);
    
    pose = odometry.update(m_Gyro.getRotation2d(), getLeftDistance(), getRightDistance());
    m_field.setRobotPose(pose);
  }
}
