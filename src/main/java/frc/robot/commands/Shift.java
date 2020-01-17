/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Shifter;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.MConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Shift extends CommandBase {

  private final Shifter shifter;
  private final Joystick joystickR;
  private final Joystick joystickL;

  public Shift(Shifter inShifter) {
    shifter = inShifter;

    joystickR = new Joystick(joyRPort);
    joystickL = new Joystick(joyLPort);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if( isInAutoShift != true ){ //if not in autonomous mode

      if ( joystickR.getRawButton(1) ) { //Shift up
        shifter.shiftUp();
      } else if ( joystickL.getRawButton(1) ) { //shift down 
        shifter.shiftDown();
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
