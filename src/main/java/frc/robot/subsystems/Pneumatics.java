/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Compressor;

public class Pneumatics extends SubsystemBase {

  public static Compressor comp;

  public Pneumatics() {
    comp = new Compressor();
  }

  public void pressurize(){
    comp.start();
  }
  public void stop(){
  	comp.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
