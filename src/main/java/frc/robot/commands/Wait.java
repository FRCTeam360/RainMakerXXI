/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Timer;

//This command is used to add a delay to an autonomous
//If needed for strategy, add it to the sequential where it is needed
//Pass it time in seconds, it will overshoot it a bit based on roborio ticks so prolly about .05 seconds overshoot on average
public class Wait extends CommandBase {

  private Timer timer;
  private double targetTime;

  public Wait(double timeToWaitInSeconds) {
    timer = new Timer();
    targetTime = timeToWaitInSeconds;
  }

  @Override   // Called when the command is initially scheduled.
  public void initialize() {
    timer.start();
  }

  @Override   // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    timer.stop();
	  timer.reset();
  }

  @Override   // Returns true when the command should end.
  public boolean isFinished() {
    return timer.get() >= targetTime;
  }
}
