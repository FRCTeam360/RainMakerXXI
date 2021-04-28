/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Falcons extends SubsystemBase {
  
  private TalonFX falcon13;

  public Falcons() {
    falcon13 = new TalonFX(13);
  }

  public void runPercent (double pPower) {
    falcon13.set(ControlMode.PercentOutput , pPower);
  }

  public void encoderPrintout() {
    SmartDashboard.putNumber("Falcon13", falcon13.getSelectedSensorPosition());
  }

  @Override
  public void periodic() {
    //encoderPrintout();
  }
}
