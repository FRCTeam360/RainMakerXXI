/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Joystick;
import static frc.robot.Constants.OIConstants.*;

import static frc.robot.Constants.inAuto;

public class ShooterRamp extends CommandBase {
  
  private final Shooter myShooter;
  Joystick cont;


  public ShooterRamp(Shooter shooter) {
    myShooter = shooter;
    cont = new Joystick(contPort);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    myShooter.run();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myShooter.runWithJoy(0); //Sets power to zero percent
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!inAuto && cont.getRawButton(3)) { //Programmed in Operator abort feature, if this command ends, alignshoot ends
      return true;
    } else {
      return false;
    }
  }
}
