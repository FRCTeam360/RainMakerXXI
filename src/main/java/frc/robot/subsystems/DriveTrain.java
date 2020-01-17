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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  public static CANSparkMax motorL1;
  public static CANSparkMax motorL2;
  public static CANSparkMax motorR1;
  public static CANSparkMax motorR2;

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


  public DriveTrain() {
    motorL1 = new CANSparkMax(1, MotorType.kBrushless);
    motorL2 = new CANSparkMax(2, MotorType.kBrushless);
    motorR1 = new CANSparkMax(3, MotorType.kBrushless);
    motorR2 = new CANSparkMax(4, MotorType.kBrushless);

    // makes the second motor for left and right sides to follow the primary motor on the left and right
    motorL2.follow(motorL1);
    motorR2.follow(motorR1);

    // makes one side of the robot reverse direction in order to ensure that the robot goes forward when the joysticks are both forward and backwards when the joysticks are both backwards
    motorL1.setInverted(false);
    motorL2.setInverted(false);
    motorR1.setInverted(true);
    motorR2.setInverted(true);

  }

  public void driveRMAX (double Rmotor) {
    motorR1.set( Rmotor );
  }
  public void driveLMAX (double Lmotor) {
    motorL1.set( Lmotor );
  }

  public void leftEnc(){
    // gets the new position of the encoder
    leftNewPos = motorL1.getEncoder().getPosition();
    // puts raw number in smartdashboard
    SmartDashboard.putNumber("Left Raw Pos", leftNewPos);
    // finds the difference in the new and the old position
    deltaLeftPos = leftNewPos - leftOldPos;
    // gets the velocity of the left motor
    leftVel = motorL2.getEncoder().getVelocity();
    // puts raw number in smartdashboard
    SmartDashboard.putNumber("Left Raw Vel", leftVel);
  }

  public void rightEnc(){
    // gets the new position of the encoder
    rightNewPos = motorR1.getEncoder().getPosition();
    // puts raw number in smartdashboard
    SmartDashboard.putNumber("Right Raw Pos", rightNewPos);
    // finds the difference in the new and the old position
    deltaRightPos = rightNewPos - rightOldPos;
    // gets the velocity of the left motor
    rightVel = motorR2.getEncoder().getVelocity();
    // puts raw number in smartdashboard
    SmartDashboard.putNumber("Right Raw Vel", rightVel);
  }

  public void brakeMode() {
    motorL1.setIdleMode(IdleMode.kBrake);
    motorR1.setIdleMode(IdleMode.kBrake);
    motorL2.setIdleMode(IdleMode.kBrake);
    motorR2.setIdleMode(IdleMode.kBrake);
  }
  public void coastMode() {
    motorL1.setIdleMode(IdleMode.kCoast);
    motorR1.setIdleMode(IdleMode.kCoast);
    motorL2.setIdleMode(IdleMode.kCoast);
    motorR2.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rightEnc();
    leftEnc();
  }
}
