/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.OIConstants.*;

public class ShootBalls extends CommandBase {

  private final Shooter myShooter;

  private final Joystick joyR;
  private final Joystick cont;

  /**
   * Creates a new ShootBalls.
   */
  public ShootBalls(Shooter shooter) {
    myShooter = shooter;
    joyR = new Joystick(joyRPort);
    cont = new Joystick(contPort);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Shooting Balls");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    if ( joyR.getRawButton(6) ) {
      myShooter.run();
    }
    */

    if(cont.getRawButton(10)) {
      myShooter.runWithJoy((-cont.getRawAxis(1)) * 0.6);
    } else {
      myShooter.runWithJoy(0);
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
