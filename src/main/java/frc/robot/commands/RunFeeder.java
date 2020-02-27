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
//import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.OIConstants.*;
//import frc.robot.Constants.ShooterConstants;

public class RunFeeder extends CommandBase {

  Feeder myFeeder;
  Joystick cont;

  public RunFeeder(Feeder inFeeder) {
    myFeeder = inFeeder;
    cont = new Joystick(contPort);
    
    addRequirements(myFeeder); //Not require myShooter so it doesn't cancel ShootBalls command
  }

  @Override   // Called when the command is initially scheduled.
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  	//5 is for run loader, 1 is for reverse, 6 is run hopper
    if (cont.getRawButton(6)) {
			if (!cont.getRawButton(1)) {
				myFeeder.runLoader(.5); //Button 5 held & button 1 isn't held, run forwards
      } else {
				myFeeder.runLoader(-.5); //Button 5 held & button 1 is held, run reverse
      }
    } else {
      myFeeder.runLoader(0.0); //Button 5 isn't held, don't run
    }

    if (cont.getRawButton(5)) {
			if (!cont.getRawButton(1)) {
				myFeeder.runHopper(.4); //Button 6 held & button 1 isn't held, run forwards
      } else {
				myFeeder.runHopper(-.4); //Button 6 held & button 1 is held, run reverse
      }
    } else {
      myFeeder.runHopper(0.0); //Button 6 isnt held, dont run
    }

  }

  @Override   // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
