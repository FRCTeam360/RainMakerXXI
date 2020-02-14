/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Feeder;

import edu.wpi.first.wpilibj.Timer;

public class LoadBalls extends CommandBase { //Used by AlignShoot command

  Feeder myFeeder;
  Timer myTimer;

  public LoadBalls(Feeder inFeeder) {
    myFeeder = inFeeder;
    myTimer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    myTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    myFeeder.runLoader(.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myFeeder.runLoader(0);
    myTimer.stop();
    myTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return myTimer.get() >= 3; //Go until 3 seconds
  }
}
