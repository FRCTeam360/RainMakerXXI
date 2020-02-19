/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS; //https://www.kauailabs.com/dist/frc/2020/navx_frc.json
import edu.wpi.first.wpilibj.SPI;

public class Navx extends SubsystemBase {

  AHRS navX;

  public Navx() {
    navX = new AHRS(SPI.Port.kMXP);
  }

  public double getHeading() {
    return Math.IEEEremainder(navX.getAngle(), 360) * (true ? -1.0 : 1.0); //True cuz oits this way in pathweav6
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("NAV Raw Angle", navX.getAngle() );
    SmartDashboard.putNumber("NAV Final Angle", getHeading() );

  }
}
