/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.subsystems.Climber;
import static frc.robot.Constants.OIConstants.*;

public class Climb extends CommandBase { //Tele-op command / no isFinsihed() method

  private final Climber myClimber;
  private final Joystick cont;
  private boolean enableClimberMotors;

  public Climb (Climber inClimber) {
    myClimber = inClimber;
    cont = new Joystick(contPort);
    enableClimberMotors = false;
    
    addRequirements(myClimber);  // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    enableClimberMotors = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (enableClimberMotors == true) { //If motors enabled

      if (Math.abs(cont.getRawAxis(1)) >= .10 && !(cont.getRawButton(10)) ) {
        myClimber.runLeftClimber( -cont.getRawAxis(1) ); 
      } else {
        myClimber.runLeftClimber(0.0);
      }

      if ( (Math.abs(cont.getRawAxis(3)) >= .10)  && !(cont.getRawButton(9)) ) {
        myClimber.runRightClimber( -cont.getRawAxis(3) ); 
      } else {
        myClimber.runRightClimber(0.0);
      }

    } else if ( cont.getRawButton(2) ) { //If they're disabled check if the bind to enable them has been hit
      enableClimberMotors = true;
    }

    if ( cont.getRawButton(9) ) {
        myClimber.runErector(-cont.getRawAxis(3) ); //If button hit, run the erector on the axis
    } else {
      myClimber.runErector(0.0);
    } 

    SmartDashboard.putBoolean("Climber Enable", enableClimberMotors);

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
