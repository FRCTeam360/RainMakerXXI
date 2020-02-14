/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {

  private static TalonSRX erector;   //One bag motor to raise it up (controlled by talon) 
  
  private static CANSparkMax motorMaster; //2 Neos mast/slav to pull robot up "climber"
  private static CANSparkMax motorSlave;

  public Climber() {
    erector = new TalonSRX(erectorMotorId);
    motorMaster = new CANSparkMax(motorMasterId, MotorType.kBrushless); //For encoders, only based on Master
    motorSlave = new CANSparkMax(motorSlaveId, MotorType.kBrushless);

    motorSlave.follow(motorMaster);

    motorMaster.setInverted(false); //This is only a guess
    motorSlave.setInverted(true);   //May change later

    motorMaster.setIdleMode(IdleMode.kBrake); //Set brake mode on the climber
  }

  public void resetClimberEncoder () {
    motorMaster.getEncoder().setPosition(0);
  }
  public double getClimberEncoderPos() {
    return motorMaster.getEncoder().getPosition();
  }
  public void runClimber (double pPower) {
    motorMaster.set(pPower);
  }

  public void resetErectorEncoder() {
    erector.setSelectedSensorPosition(0);
  }
  public double getErectorEncoderPos() {
    return erector.getSelectedSensorPosition();
  }
  public void runErector(double pPower) {
    erector.set(ControlMode.PercentOutput, pPower);
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run - Velocity & Position of each motor
    SmartDashboard.putNumber("Climber Pos" , motorMaster.getEncoder().getPosition() );
    SmartDashboard.putNumber("Climber Vel", motorMaster.getEncoder().getVelocity() );
    SmartDashboard.putNumber("Erector Pos", erector.getSelectedSensorPosition() );
    SmartDashboard.putNumber("Erector Vel", erector.getSelectedSensorVelocity() );
  }
}
