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

  private final Joystick joy;

  /**
   * Creates a new ShootBalls.
   */
  public ShootBalls(Shooter shooter) {
    myShooter = shooter;
    joy = new Joystick(joy1Port);
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
    if ( joy.getRawButton(1) ) {

      //shooterMaster.set(ControlMode.Velocity , (((Constants.targetRpm * 4096) / 600) / 2) ); //divided by 2 is for our gear ratio
      //System.out.println( (( (Constants.targetRpm * 4096) / 600) / 2) + "   " );    //about= 13,650
      myShooter.run();

    } else if(joy.getRawButton(6)) {

      myShooter.runWithJoy((-joy.getRawAxis(1)) * 0.6);

    } else {
      myShooter.runWithJoy(0);
      //shooterMaster.set(ControlMode.PercentOutput , joy.getRawAxis(1) );
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
