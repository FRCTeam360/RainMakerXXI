/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Talons extends SubsystemBase {
  
  private TalonSRX talon7;
  private TalonSRX talon8;
  private TalonSRX talon9;
  private TalonSRX talon10;
  private TalonSRX talon11;
  private TalonSRX talon12;

  public Talons() {
    talon7 = new TalonSRX(7);
    talon8 = new TalonSRX(8);
    talon9 = new TalonSRX(9);
    talon10 = new TalonSRX(10);
    talon11 = new TalonSRX(11);
    talon12 = new TalonSRX(12);
  }

  public void runPercent (double pPower) {
    talon7.set(ControlMode.PercentOutput , pPower);
    talon8.set(ControlMode.PercentOutput , pPower);
    talon9.set(ControlMode.PercentOutput , pPower);
    talon10.set(ControlMode.PercentOutput , pPower);
    talon11.set(ControlMode.PercentOutput , pPower);
    talon12.set(ControlMode.PercentOutput , pPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
