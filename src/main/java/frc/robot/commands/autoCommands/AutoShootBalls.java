/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.ShooterConstants;

public class AutoShootBalls extends CommandBase {

  private final Shooter myShooter;
  private final Feeder myFeeder;
  private final Timer timer;

  public AutoShootBalls(Shooter shooter, Feeder feeder) {
    myShooter = shooter;
    myFeeder = feeder;
    timer = new Timer();
    addRequirements(shooter, feeder);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    myShooter.run();
    if ( myShooter.getVelocity() > ShooterConstants.targetVelocity - 500 ){
      myFeeder.runLoader(.5);
      timer.start();
    }
  }

  @Override
  public void end(boolean interrupted) { // Called once the command ends or is interrupted.
    myShooter.runWithJoy(0);
    myFeeder.runLoader(0);

    timer.stop();
    timer.reset();
  }

  @Override
  public boolean isFinished() {   // Returns true when the command should end.
    if ( timer.get() >= 3) { //How long the feeder needs to run for to feed all three balls
      return true;
    } else {
      return false;
    }
  }

}
