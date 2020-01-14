/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Shooter extends SubsystemBase {

  public int targetShooterRPM;
  public TalonSRX shooterMaster;
  public TalonSRX shooterSlave;

  public Shooter(int rmp) {
    shooterMaster = new TalonSRX(0);
    shooterSlave = new TalonSRX(1);

    targetShooterRPM = rmp;
  }

  public void start () {
    //double current = shooterMaster.getStatorCurrent (); //amps
    //int rawVelocity = shooterMaster.getSelectedSensorVelocity(); // raw sensor units
    //shooterMaster.set( ControlMode.PercentOutput , 1 );
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
