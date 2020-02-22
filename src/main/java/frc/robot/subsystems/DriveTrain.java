/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.DriveTrainConstants.*;
import frc.robot.Constants.AutoConstants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import com.kauailabs.navx.frc.AHRS; //If error here check updates: install vendor online use: https://www.kauailabs.com/dist/frc/2020/navx_frc.json
import edu.wpi.first.wpilibj.SPI; //Port NavX is on

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private static CANSparkMax motorLMaster;
  private static CANSparkMax motorLSlave;
  private static CANSparkMax motorRMaster;
  private static CANSparkMax motorRSlave;

  private final DifferentialDrive m_differentialDrive;

  private double leftVel;   // initializes velocities for left and right sides
  private double rightVel;
  private double leftNewPos;   // initializes new positions for left and right sides
  private double rightNewPos;

  private AHRS navX;
  private final DifferentialDriveOdometry m_odometry;
  private final SpeedControllerGroup leftGroup;
  private final SpeedControllerGroup rightGroup;

  public DriveTrain() {
    motorLMaster = new CANSparkMax(motorLMasterID, MotorType.kBrushless);
    motorLSlave = new CANSparkMax(motorLSlaveID, MotorType.kBrushless);
    motorRMaster = new CANSparkMax(motorRMasterID, MotorType.kBrushless);
    motorRSlave = new CANSparkMax(motorRSlaveID, MotorType.kBrushless);

    motorLMaster.restoreFactoryDefaults();
    motorLSlave.restoreFactoryDefaults();
    motorRMaster.restoreFactoryDefaults();
    motorRSlave.restoreFactoryDefaults();

    // makes the second motor for left and right sides to follow the primary motor on the left and right
    motorLSlave.follow(motorLMaster);
    motorRSlave.follow(motorRMaster);

    // makes one side of the robot reverse direction in order to ensure that the robot goes forward when the joysticks are both forward and backwards when the joysticks are both backwards
    motorLMaster.setInverted(false);
    motorLSlave.setInverted(false);
    motorRMaster.setInverted(true);
    motorRSlave.setInverted(true);

    navX = new AHRS(SPI.Port.kMXP); //For frc-characterization tool: "SPI.Port.kMXP" of type "NavX"
    resetEncPos(); //Reset Encoders r navX yaw before m_odometry is defined 
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    motorLMaster.getEncoder().setVelocityConversionFactor(1/42.0); //42 is encoder resolution
    motorRMaster.getEncoder().setVelocityConversionFactor(1/42.0);

    leftGroup = new SpeedControllerGroup( motorLMaster , motorLSlave );
    rightGroup = new SpeedControllerGroup( motorRMaster , motorRSlave );

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("steer", steer);
    SmartDashboard.putNumber("maxDrive", maxDrive);

    m_differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
    m_differentialDrive.setSafetyEnabled(false);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftGroup.setVoltage(leftVolts); //Answer is no   //Set to motor groups
    rightGroup.setVoltage(rightVolts); //it's big brain time
    m_differentialDrive.feed();
  }

  public void resetEncPos () { //For initialization resets encoder positions, for ramsete
    motorLMaster.getEncoder().setPosition(0);
    motorRMaster.getEncoder().setPosition(0);
    navX.zeroYaw();
    m_odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(getHeading())); //Set odomentry to zero
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

  public DifferentialDriveWheelSpeeds getWheelSpeeds() { //Must be in meters/second
    return new DifferentialDriveWheelSpeeds(
      motorLMaster.getEncoder().getVelocity() * AutoConstants.ticksToMeters * AutoConstants.hundredMstoSecond,
      motorRMaster.getEncoder().getVelocity() * AutoConstants.ticksToMeters * AutoConstants.hundredMstoSecond
    ); //In example: m_leftEncoder.getRate() , m_rightEncoder.getRate()
  }

  public void leftEnc(){
    leftNewPos = motorLMaster.getEncoder().getPosition();     // gets the new position of the encoder
    SmartDashboard.putNumber("Left Raw Pos", leftNewPos);     // puts raw number in smartdashboard
    leftVel = motorLMaster.getEncoder().getVelocity();     // gets the velocity of the left motor
    SmartDashboard.putNumber("Left Raw Vel", leftVel);     // puts raw number in smartdashboard
  }

  public void rightEnc(){
    rightNewPos = motorRMaster.getEncoder().getPosition();     // gets the new position of the encoder
    SmartDashboard.putNumber("Right Raw Pos", rightNewPos);     // puts raw number in smartdashboard
    rightVel = motorRMaster.getEncoder().getVelocity();     // gets the velocity of the left motor
    SmartDashboard.putNumber("Right Raw Vel", rightVel);     // puts raw number in smartdashboard
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

  private void PIDDashboard() {
    // read PID coefficients from SmartDashboard
    double s = SmartDashboard.getNumber("steer", 0);
    double max = SmartDashboard.getNumber("maxDrive", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((s != steer)) { steer = s; }
    if((max != maxDrive)) { maxDrive = max; }
  }

  public void navxTestingDashboardReadouts () {
    //SmartDashboard.putNumber("N ang", Math.IEEEremainder(navX.getAngle(), 360) );
    SmartDashboard.putNumber("NAV ang", navX.getAngle() );
    SmartDashboard.putString("Pos2D",  m_odometry.getPoseMeters().toString() );
    //SmartDashboard.putNumber("N pre", navX.getBarometricPressure()); //why this no work cri, just tryna get the pressure
    //SmartDashboard.putNumber("N yaw", navX.getYaw());

    //SmartDashboard.putBoolean("NAVC con", navX.isConnected());
    //SmartDashboard.putBoolean("NAV cal", navX.isCalibrating());
  }

  public double avgMeterTrav() { //Used exclusively by: AutoBackupOnTicks
    return ((motorLMaster.getEncoder().getPosition() * AutoConstants.ticksToMeters) + (motorRMaster.getEncoder().getPosition() * AutoConstants.ticksToMeters)) / 2;
  }

  public double getHighestVelocity () { 
    double leftSpeed = motorLMaster.getEncoder().getVelocity() * AutoConstants.ticksToMeters * AutoConstants.hundredMstoSecond;
    double rightSpeed = motorRMaster.getEncoder().getVelocity() * AutoConstants.ticksToMeters * AutoConstants.hundredMstoSecond;
    double highSpeed = Math.max( Math.abs(leftSpeed), Math.abs(rightSpeed) ); //Make em both positive
    return highSpeed; //In meters per second
  }

  public void tempPrintouts() {
    SmartDashboard.putNumber("LM Temp", motorLMaster.getMotorTemperature() );
    SmartDashboard.putNumber("LS Temp", motorLSlave.getMotorTemperature() );
    SmartDashboard.putNumber("RM Temp", motorRMaster.getMotorTemperature() );
    SmartDashboard.putNumber("RS Temp", motorRSlave.getMotorTemperature() );
  }

  public void avgTempPrintouts() {
    double total = 
      motorLMaster.getMotorTemperature() + 
      motorLSlave.getMotorTemperature() + 
      motorRMaster.getMotorTemperature() +
      motorRSlave.getMotorTemperature() ;
    double avg = total / 4;
    SmartDashboard.putNumber("Avg Drive Temp", avg);
  }

  public void ampPrintouts() {
    SmartDashboard.putNumber("LM Amp", motorLMaster.getOutputCurrent() );
    SmartDashboard.putNumber("LS Amp", motorLSlave.getOutputCurrent() );
    SmartDashboard.putNumber("RM Amp", motorRMaster.getOutputCurrent() );
    SmartDashboard.putNumber("RS Amp", motorRSlave.getOutputCurrent() );
  }

  public void dashboardMetersTravelled() {
    SmartDashboard.putNumber("Left Meters", motorLMaster.getEncoder().getPosition() * AutoConstants.ticksToMeters);
    SmartDashboard.putNumber("Right Meters", motorRMaster.getEncoder().getPosition() * AutoConstants.ticksToMeters);
    //SmartDashboard.putNumber("Num", AutoConstants.ticksToMeters);
    SmartDashboard.putNumber("Left m/s", motorLMaster.getEncoder().getVelocity() * AutoConstants.ticksToMeters * AutoConstants.hundredMstoSecond);
    SmartDashboard.putNumber("Right m/s", motorRMaster.getEncoder().getVelocity() * AutoConstants.ticksToMeters * AutoConstants.hundredMstoSecond);
  }

  @Override
  public void periodic() {
    m_odometry.update( //Must be in meters according to internets
      Rotation2d.fromDegrees(getHeading()),
      motorLMaster.getEncoder().getPosition() * AutoConstants.ticksToMeters,
      motorRMaster.getEncoder().getPosition() * AutoConstants.ticksToMeters
    );

    //rightEnc();
    //leftEnc();
    PIDDashboard(); //For mathew's PIDs
    navxTestingDashboardReadouts(); //Here for testing
    dashboardMetersTravelled();
    tempPrintouts();
    SmartDashboard.putNumber("Cons", AutoConstants.ticksToMeters);
    //avgTempPrintouts();
    ampPrintouts();
  }
}
