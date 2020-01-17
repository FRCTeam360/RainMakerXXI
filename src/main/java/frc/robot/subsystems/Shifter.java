/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Shifter extends SubsystemBase {

  public static DoubleSolenoid shifter;
  public static enum ShiftState {UP, DOWN, UNKNOWN};
  public static ShiftState shiftState;

  public Shifter() {
    shifter = new DoubleSolenoid(1, 0);
    shiftState = ShiftState.UNKNOWN;
  }

	public void shiftUp(){ 
		shifter.set(DoubleSolenoid.Value.kForward);
		shiftState = ShiftState.UP;
	}
	public void shiftDown() {
		shifter.set(DoubleSolenoid.Value.kReverse);
		shiftState = ShiftState.DOWN;
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
