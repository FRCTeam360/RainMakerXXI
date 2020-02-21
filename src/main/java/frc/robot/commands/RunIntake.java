/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
  
  private final Intake myIntake;
  private final Joystick cont;

  public RunIntake(Intake intake) {
    cont = new Joystick(contPort);
    myIntake = intake;
    addRequirements(myIntake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //X is 1 && 7 is Left Trigger
    if ( cont.getRawButton(7) ) {
      if (!cont.getRawButton(1)) {
				myIntake.run(0.85); //If left trigger is pressed and x isn't pressed, run forward
      } else {
				myIntake.run(-0.85); //If left trigger is pressed and x is pressed, run backwards
      }
    } else {
      myIntake.run(0.0); //If 7 isn't hit, stop it
    }
    
    /* //Disables old binds for intake
    if(joyOI.getRawButton(6)){
      myIntake.run(.85); //Full power was too much according to the vande boy
    }else{
      myIntake.run(0);
    }
    */
    /* //Disables the pneumatic
    if (joyOI.getRawButton(5)){
      myIntake.intakeUp();
    }else if (joyOI.getRawButton(6)){
      myIntake.intakeDown();
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
