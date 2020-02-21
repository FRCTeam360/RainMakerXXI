/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;

import frc.robot.Constants.ShooterConstants;

public class ShootBalls extends CommandBase {

  private final Shooter myShooter;
  private final Feeder myFeeder;

  public ShootBalls(Shooter shooter, Feeder feeder) {
    myShooter = shooter;
    myFeeder = feeder;
    addRequirements(myShooter, myFeeder);
  }

  @Override // Called when the command is initially scheduled.
  public void initialize() { }

  @Override
  public void execute() {   // Called every time the scheduler runs while the command is scheduled.
    myShooter.run();
    if ( myShooter.getVelocity() > ShooterConstants.targetVelocity - 500 ){
      myFeeder.runLoader(0.5); //Half speed forward
      myFeeder.runHopper(0.85); //Forward almost full speed
    }
  }

  @Override
  public void end(boolean interrupted) {   // Called once the command ends or is interrupted. //When the button is released
    myFeeder.runBoth(0.0); //Stop loader & Feeder 
    myShooter.runWithJoy(0.0); //Stop the shooter
  }

  @Override // Returns true when the command should end. //Stops when it's key it's bound to in robotContainer let go
  public boolean isFinished() { return false; }
}
