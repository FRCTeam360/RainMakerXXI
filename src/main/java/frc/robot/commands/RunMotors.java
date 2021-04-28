/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Joystick;
import static frc.robot.Constants.OIConstants.*;

import frc.robot.subsystems.Talons;
import frc.robot.subsystems.SparkMaxes;
import frc.robot.subsystems.Falcons;

public class RunMotors extends CommandBase {

  private final Joystick joyR;
  private final Joystick joyL;

  private final Talons myTalons;
  private final SparkMaxes mySparkMaxes;
  private final Falcons myFalcons;

  public RunMotors(Talons talons, SparkMaxes sparkMaxes, Falcons falcons ) {
    myTalons = talons;
    mySparkMaxes = sparkMaxes;
    myFalcons = falcons;

    joyR = new Joystick(joyRPort);
    joyL = new Joystick(joyLPort);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myTalons, mySparkMaxes);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Math.abs(joyR.getRawAxis(1)) >= .15){ //Right side moved
    	mySparkMaxes.runPercent(-1 * joyR.getRawAxis(1) * 0.8);
    }else{
    	mySparkMaxes.runPercent(0);
    }

    if(Math.abs(joyL.getRawAxis(1)) >= .15){ //Left side moved
      myTalons.runPercent(-1 * joyL.getRawAxis(1) * 0.8);
    }else{
    	myTalons.runPercent(0);
    }

    /* //Uncomment if you want to use the Falcons
    if () {
      myFalcons.runPercent(-1 * joyL.getRawAxis(1) * 0.8);
    } else {
      myFalcons.runPercent(0);
    }
    */

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
