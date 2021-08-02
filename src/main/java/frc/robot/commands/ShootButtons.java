/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.OIConstants.*;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.subsystems.Shooter;

public class ShootButtons extends CommandBase {

  private final Shooter shooter;
  private final Joystick cont;

  public ShootButtons(Shooter shooter) {
    this.shooter = shooter;
    cont = new Joystick(contPort);
    addRequirements(shooter);// Use addRequirements() here to declare subsystem dependencies.
  }

  @Override   // Called when the command is initially scheduled.
  public void initialize() {
  }

  @Override   // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    if (cont.getRawButton( 2 )) { //If A button held
      shooter.setRun( 1500.0 ); //if button held, run
    } else if (cont.getRawButton(3)){
      shooter.setRun(1600.0);
    } else if (cont.getRawButton(4)){
      shooter.setRun(1700.0);
    } else {
      shooter.runWithJoy(0.0); //Don't run
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
