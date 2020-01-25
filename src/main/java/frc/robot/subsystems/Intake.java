/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */

  private TalonSRX intake;
  private DoubleSolenoid intakeMover;

  public Intake() {
    intake = new TalonSRX(intakeId);
    intakeMover = new DoubleSolenoid(forwardChannel, reverseChannel);
  }

  //Intake motor speed
  public void run (double speed){
    intake.set(ControlMode.PercentOutput, speed);
  }

  //Moves intake up
  public void intakeUp(){
    intakeMover.set(DoubleSolenoid.Value.kForward);
  }

  //Moves intake down
  public void intakeDown(){
    intakeMover.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
