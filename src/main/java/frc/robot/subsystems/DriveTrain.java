/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.DriveTrainConstants.*;
import frc.robot.Constants.AutoConstants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private static CANSparkMax motorLMaster;
  private static CANSparkMax motorLSlave;
  private static CANSparkMax motorRMaster;
  private static CANSparkMax motorRSlave;

  public CANEncoder rightOne;
  public CANEncoder rightTwo;
  public CANEncoder leftOne;
  public CANEncoder leftTwo;

  // initializes velocities for left and right sides
  double leftVel;
  double rightVel;
  // initializes new positions for left and right sides
  double leftNewPos;
  double rightNewPos;
  // initializes old position values to zero
  double leftOldPos = 0;
  double rightOldPos = 0;
  // initializes the output variable to zero for left and right sides
  double leftOutput = 0;
  double rightOutput = 0;
  // initializes the changes in positions for left and right sides
  double deltaRightPos;
  double deltaLeftPos;

  AHRS navX;
  private final DifferentialDriveOdometry m_odometry;

  public DriveTrain() {
    motorLMaster = new CANSparkMax(motorLMasterID, MotorType.kBrushless);
    motorLSlave = new CANSparkMax(motorLSlaveID, MotorType.kBrushless);
    motorRMaster = new CANSparkMax(motorRMasterID, MotorType.kBrushless);
    motorRSlave = new CANSparkMax(motorRSlaveID, MotorType.kBrushless);

    // makes the second motor for left and right sides to follow the primary motor on the left and right
    motorLSlave.follow(motorLMaster);
    motorRSlave.follow(motorRMaster);

    // makes one side of the robot reverse direction in order to ensure that the robot goes forward when the joysticks are both forward and backwards when the joysticks are both backwards
    motorLMaster.setInverted(false);
    motorLSlave.setInverted(false);
    motorRMaster.setInverted(true);
    motorRSlave.setInverted(true);

    navX = new AHRS(SPI.Port.kMXP); //For frc-characterization tool: "SPI.Port.kMXP" of type "navX"
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    motorRMaster.setVoltage(leftVolts);
    motorLMaster.setVoltage(-rightVolts); //Does this need to be inverted? 
  }

  public double getHeading() {
    return Math.IEEEremainder(navX.getAngle(), 360) * (AutoConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void driveRMAX (double Rmotor) {
    motorRMaster.set( Rmotor );
  }
  public void driveLMAX (double Lmotor) {
    motorLMaster.set( Lmotor );
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds( motorLMaster.getEncoder().getVelocity() , motorRMaster.getEncoder().getVelocity() ); //m_leftEncoder.getRate() , m_rightEncoder.getRate()
  }

  public void leftEnc(){
    // gets the new position of the encoder
    leftNewPos = motorLMaster.getEncoder().getPosition();
    // puts raw number in smartdashboard
    SmartDashboard.putNumber("Left Raw Pos", leftNewPos);
    // finds the difference in the new and the old position
    deltaLeftPos = leftNewPos - leftOldPos;
    // gets the velocity of the left motor
    leftVel = motorLSlave.getEncoder().getVelocity();
    // puts raw number in smartdashboard
    SmartDashboard.putNumber("Left Raw Vel", leftVel);
  }

  public void rightEnc(){
    // gets the new position of the encoder
    rightNewPos = motorRMaster.getEncoder().getPosition();
    // puts raw number in smartdashboard
    SmartDashboard.putNumber("Right Raw Pos", rightNewPos);
    // finds the difference in the new and the old position
    deltaRightPos = rightNewPos - rightOldPos;
    // gets the velocity of the left motor
    rightVel = motorRSlave.getEncoder().getVelocity();
    // puts raw number in smartdashboard
    SmartDashboard.putNumber("Right Raw Vel", rightVel);
  }

  public void brakeMode() {
    motorLMaster.setIdleMode(IdleMode.kBrake);
    motorRMaster.setIdleMode(IdleMode.kBrake);
    motorLSlave.setIdleMode(IdleMode.kBrake);
    motorRSlave.setIdleMode(IdleMode.kBrake);
  }
  public void coastMode() {
    motorLMaster.setIdleMode(IdleMode.kCoast);
    motorRMaster.setIdleMode(IdleMode.kCoast);
    motorLSlave.setIdleMode(IdleMode.kCoast);
    motorRSlave.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), motorLMaster.getEncoder().getPosition(), motorRMaster.getEncoder().getPosition() );

    rightEnc();
    leftEnc();
  }
}
