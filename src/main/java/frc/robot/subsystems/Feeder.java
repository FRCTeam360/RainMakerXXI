/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FeederConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;



public class Feeder extends SubsystemBase {
  /**
   * Creates a new Feeder.
   */

  private TalonSRX loader;
  private TalonSRX hopper;

  public Feeder() {
    loader = new TalonSRX(loaderMotorId);
    hopper = new TalonSRX(hopperMotorId);
  }

  public void runHopper (double speed) {
    hopper.set(ControlMode.PercentOutput, speed);
    
  }

  public void runLoader (double speed) {
    loader.set(ControlMode.PercentOutput, speed);
    
  }

  public void runBoth (double speed) {
    loader.set(ControlMode.PercentOutput, speed);
    hopper.set(ControlMode.PercentOutput, speed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
