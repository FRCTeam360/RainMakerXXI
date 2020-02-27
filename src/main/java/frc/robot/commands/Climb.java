/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.subsystems.Climber;

import static frc.robot.Constants.OIConstants.*;

public class Climb extends CommandBase { //Tele-op command / no isFinsihed() method

  private final Climber myClimber;
  private final Joystick cont;

  public Climb (Climber inClimber) {
    myClimber = inClimber;
    cont = new Joystick(contPort);
    
    addRequirements(myClimber);  // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(cont.getRawAxis(1)) >= .10 && !(cont.getRawButton(10)) ) {
			myClimber.runLeftClimber( -cont.getRawAxis(1) ); //If direction wrong, modify in Climber subsystem
    } else {
			myClimber.runLeftClimber(0.0);
    }

    if ( (Math.abs(cont.getRawAxis(3)) >= .10)  && !(cont.getRawButton(10)) ) {
			myClimber.runRightClimber( -cont.getRawAxis(3) ); //If direction wrong, modify in Climber subsystem
    } else {
			myClimber.runRightClimber(0.0);
    }

    if ( cont.getRawButton(10) ) {
        myClimber.runErector(-cont.getRawAxis(3) ); //If button hit, run the erector on the axis
    } else {
      myClimber.runErector(0.0);
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
