/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.DriveTrainConstants.*;
import frc.robot.Constants.AutoConstants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import com.kauailabs.navx.frc.AHRS; //If error here check updates: install vendor online use: https://www.kauailabs.com/dist/frc/2020/navx_frc.json
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private static CANSparkMax motorLMaster;
  private static CANSparkMax motorLSlave;
  private static CANSparkMax motorRMaster;
  private static CANSparkMax motorRSlave;

  private CANPIDController pidControllerLeft;
  private CANPIDController pidControllerRight;

  public CANEncoder rightMaster;
  public CANEncoder rightSlave;
  public CANEncoder leftMaster;
  public CANEncoder leftSlave;

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

    motorLMaster.restoreFactoryDefaults();
    motorLSlave.restoreFactoryDefaults();
    motorRMaster.restoreFactoryDefaults();
    motorRSlave.restoreFactoryDefaults();

    pidControllerLeft = motorLMaster.getPIDController();
    pidControllerRight = motorRMaster.getPIDController();

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

    rightMaster = motorRMaster.getEncoder();
    leftMaster = motorLMaster.getEncoder();

    pidControllerLeft.setP(kPLeft);
    pidControllerLeft.setI(kILeft);
    pidControllerLeft.setD(kDLeft);
    pidControllerLeft.setIZone(kIzLeft);
    pidControllerLeft.setFF(kFFLeft);

    pidControllerRight.setP(kPRight);
    pidControllerRight.setI(kIRight);
    pidControllerRight.setD(kDRight);
    pidControllerRight.setIZone(kIzRight);
    pidControllerRight.setFF(kFFRight);

    pidControllerLeft.setOutputRange(kMinOutput, kMaxOutput);
    pidControllerRight.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain Left", kPLeft);
    SmartDashboard.putNumber("I Gain Left", kILeft);
    SmartDashboard.putNumber("D Gain Left", kDLeft);
    SmartDashboard.putNumber("I Zone Left", kIzLeft);
    SmartDashboard.putNumber("Feed Forward Left", kFFLeft);

    SmartDashboard.putNumber("P Gain Right", kPRight);
    SmartDashboard.putNumber("I Gain Right", kIRight);
    SmartDashboard.putNumber("D Gain Right", kDRight);
    SmartDashboard.putNumber("I Zone Right", kIzRight);
    SmartDashboard.putNumber("Feed Forward Right", kFFRight);
    
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    motorRMaster.setVoltage(leftVolts);
    motorLMaster.setVoltage(rightVolts); //Answer is no
  }

  public void resetEncPos () { //For initialization resets encoder positions, for ramsete
    motorLMaster.getEncoder().setPosition(0);
    motorRMaster.getEncoder().setPosition(0);
    navX.zeroYaw();
    //navX.setAngleAdjustment(-90); //I think we need to do this, https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html & https://pdocs.kauailabs.com/navx-mxp/installation/orientation-2/
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
  
  public void velocityDrive (double setPoint) {
    pidControllerLeft.setReference(setPoint, ControlType.kVelocity);
    pidControllerRight.setReference(setPoint, ControlType.kVelocity); //Potentially we need to invert this
  }

  public void leftEnc(){
    leftNewPos = motorLMaster.getEncoder().getPosition();     // gets the new position of the encoder
    SmartDashboard.putNumber("Left Raw Pos", leftNewPos);     // puts raw number in smartdashboard
    deltaLeftPos = leftNewPos - leftOldPos;     // finds the difference in the new and the old position
    leftVel = motorLMaster.getEncoder().getVelocity();     // gets the velocity of the left motor
    SmartDashboard.putNumber("Left Raw Vel", leftVel);     // puts raw number in smartdashboard
  }

  public void rightEnc(){
    rightNewPos = motorRMaster.getEncoder().getPosition();     // gets the new position of the encoder
    SmartDashboard.putNumber("Right Raw Pos", rightNewPos);     // puts raw number in smartdashboard
    deltaRightPos = rightNewPos - rightOldPos;     // finds the difference in the new and the old position
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
    double pLeft = SmartDashboard.getNumber("Left P Gain", 0);
    double iLeft = SmartDashboard.getNumber("Left I Gain", 0);
    double dLeft = SmartDashboard.getNumber("Left D Gain", 0);
    double izLeft = SmartDashboard.getNumber("Left I Zone", 0);
    double ffLeft = SmartDashboard.getNumber("Left Feed Forward", 0);
  
    double pRight = SmartDashboard.getNumber("Rigth P Gain", 0);
    double iRight = SmartDashboard.getNumber("Rigth I Gain", 0);
    double dRight = SmartDashboard.getNumber("Rigth D Gain", 0);
    double izRight = SmartDashboard.getNumber("Rigth I Zone", 0);
    double ffRight = SmartDashboard.getNumber("Rigth Feed Forward", 0);

    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((pLeft != kPLeft)) { pidControllerLeft.setP(pLeft); kPLeft = pLeft; }
    if((iLeft != kILeft)) { pidControllerLeft.setI(iLeft); kILeft = iLeft; }
    if((dLeft != kDLeft)) { pidControllerLeft.setD(dLeft); kDLeft = dLeft; }
    if((izLeft != kIzLeft)) { pidControllerLeft.setIZone(izLeft); kIzLeft = izLeft; }
    if((ffLeft != kFFLeft)) { pidControllerLeft.setFF(ffLeft); kFFLeft = ffLeft; }

    if((pRight != kPRight)) { pidControllerRight.setP(pRight); kPRight = pRight; }
    if((iRight != kIRight)) { pidControllerRight.setI(iRight); kIRight = iRight; }
    if((dRight != kDRight)) { pidControllerRight.setD(dRight); kDRight = dRight; }
    if((izRight != kIzRight)) { pidControllerRight.setIZone(izRight); kIzRight = izRight; }
    if((ffRight != kFFRight)) { pidControllerRight.setFF(ffRight); kFFRight = ffRight; }

    if((max != kMaxOutput) || (min != kMinOutput)) { 
      pidControllerLeft.setOutputRange(min, max);
      pidControllerRight.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max;
    } 
  }

  public void navxTestingDashboardReadouts () {
    SmartDashboard.putNumber("NAV1", Math.IEEEremainder(navX.getAngle(), 360) );
    SmartDashboard.putNumber("NAV2", navX.getAngle() );

    SmartDashboard.putBoolean("NAVC con", navX.isConnected());
    SmartDashboard.putBoolean("NAV cal", navX.isCalibrating());
  }

  @Override
  public void periodic() {
    m_odometry.update( //Must be in meters according to internets
      Rotation2d.fromDegrees(getHeading()),
      motorLMaster.getEncoder().getPosition() * AutoConstants.ticksToMeters,
      motorRMaster.getEncoder().getPosition() * AutoConstants.ticksToMeters
    );

    rightEnc();
    leftEnc();
    PIDDashboard();
    //navxTestingDashboardReadouts(); //Here for testing
  }
}
