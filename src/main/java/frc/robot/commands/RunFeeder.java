/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.subsystems.Feeder;

import static frc.robot.Constants.OIConstants.*;


public class RunFeeder extends CommandBase {

  Feeder myFeeder;
  Joystick cont;

  public RunFeeder(Feeder inFeeder) {
    myFeeder = inFeeder;
    cont = new Joystick(contPort);
    
    addRequirements(myFeeder);  
  }


  @Override   // Called when the command is initially scheduled.
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ( cont.getRawButton(7) && cont.getRawButton(8) ) { //Run both
      myFeeder.runLoader(.5);
      myFeeder.runHopper(.5);
    } else if ( cont.getRawButton(7) ) { //run loader, hopper to none
      myFeeder.runLoader(.5);
      myFeeder.runHopper(0);
    } else if ( cont.getRawButton(8) ) { //run hopper, loader to none
      myFeeder.runLoader(0);
      myFeeder.runHopper(.5);
    } else { //Set both to none
      myFeeder.runLoader(0);
      myFeeder.runHopper(0);
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
