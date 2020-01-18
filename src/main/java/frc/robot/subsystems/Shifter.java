/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.ShifterConstants.*;

public class Shifter extends SubsystemBase {

  public static DoubleSolenoid shifter;

  public Shifter() {
    shifter = new DoubleSolenoid( forwardChannel , reverseChannel );
    shiftState = ShiftState.UNKNOWN;
    SmartDashboard.putString( "Shifter State" , shiftState.name() );
  }

	public void shiftUp(){ 
		shifter.set(DoubleSolenoid.Value.kForward);
    shiftState = ShiftState.UP;
    SmartDashboard.putString( "Shifter State" , shiftState.name() );
	}
	public void shiftDown() {
		shifter.set(DoubleSolenoid.Value.kReverse);
    shiftState = ShiftState.DOWN;
    SmartDashboard.putString( "Shifter State" , shiftState.name() );
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
