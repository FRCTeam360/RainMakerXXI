/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Solenoids;

import edu.wpi.first.wpilibj.Joystick;
import static frc.robot.Constants.OIConstants.*;

public class FireSolenoids extends CommandBase {

  private final Solenoids mySol;
  private final Joystick cont;

  public FireSolenoids(Solenoids inSol) {
    mySol = inSol;
    cont = new Joystick(contPort);
    addRequirements(mySol);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (cont.getRawButton(5) ) { //Left bumper
      mySol.upS1();
    } else if (cont.getRawButton(6)) { //Right bumper
      mySol.downS1();
    }

    if ( cont.getRawButton(7) ) { //left trig
      mySol.upS2();
    } else if ( cont.getRawButton(8) ) { //right trig
      mySol.downS2();
    }

    if ( cont.getRawButton(9) ) { //back
      mySol.upS3();
    } else if ( cont.getRawButton(10) ) { //start
      mySol.downS3();
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
