/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {

  private static TalonSRX erector;   //One bag motor to raise it up (controlled by talon) 
  
  //private static CANSparkMax motorMaster; //2 Neos mast/slav to pull robot up "climber"
  //private static CANSparkMax motorSlave;
  private static CANSparkMax motorLeft;
  private static CANSparkMax motorRight;

  public Climber() {
    erector = new TalonSRX(erectorMotorId);
    motorLeft = new CANSparkMax(motorLeftId, MotorType.kBrushless); //For encoders, only based on Master
    motorRight = new CANSparkMax(motorRightId, MotorType.kBrushless);

    motorLeft.setInverted(false); //Currently Unkown - this is a guess
    motorRight.setInverted(true);   

    motorLeft.setIdleMode(IdleMode.kBrake); //Set brake mode on the climber
    motorRight.setIdleMode(IdleMode.kBrake);
  }

  public void runLeftClimber (double pPow) { motorLeft.set(pPow); }
  public void runRightClimber (double pPow) { motorRight.set(pPow); }
  public void runErector(double pPower) { erector.set(ControlMode.PercentOutput, pPower); }

  public void resetClimberEncoders() { motorLeft.getEncoder().setPosition(0); motorRight.getEncoder().setPosition(0); }
  public void resetErectorEncoder() { erector.setSelectedSensorPosition(0); }

  public double getErectorEncoderPos() { return erector.getSelectedSensorPosition(); }
  public double getLeftPos () { return motorLeft.getEncoder().getPosition(); }
  public double getRightPos() { return motorRight.getEncoder().getPosition(); }

  public void printouts() {
    SmartDashboard.putNumber("LC Temp", motorLeft.getMotorTemperature() );
    SmartDashboard.putNumber("RC Temp", motorRight.getMotorTemperature() );
    SmartDashboard.putNumber("LC Temp", motorLeft.getOutputCurrent() );
    SmartDashboard.putNumber("RC Temp", motorRight.getOutputCurrent() );
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run - Velocity & Position of each motor
    SmartDashboard.putNumber("Left Climber", motorLeft.getEncoder().getPosition() ); //Testing
    SmartDashboard.putNumber("Right Climber", motorRight.getEncoder().getPosition() ); //Testing
    //SmartDashboard.putNumber("Erector", erector.getSelectedSensorPosition() );
    //printouts();
  }
}
