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
    addRequirements(myShooter);    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {   // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {   // Called every time the scheduler runs while the command is scheduled.
    myShooter.run();
  }

  @Override
  public void end(boolean interrupted) {   // Called once the command ends or is interrupted.
    myShooter.runWithJoy(0); //Sets power to zero percent
  }

  @Override
  public boolean isFinished() {   // Returns true when the command should end.
    return false;
  }
}
