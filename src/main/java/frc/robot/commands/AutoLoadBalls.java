/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.ShooterConstants;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

//Used by AlignShoot command in autonomous
//Take in a time to run
public class AutoLoadBalls extends CommandBase { 

  private Feeder myFeeder;
  private Limelight myLimelight;
  private Shooter myShooter;
  private Timer myTimer;
  private double amountOfTime;

  public AutoLoadBalls(Feeder inFeeder, Limelight limelight, Shooter shooter, double amountOfTimeIn) {
    myFeeder = inFeeder;
    myLimelight = limelight;
    myShooter = shooter;
    amountOfTime = amountOfTimeIn;

    myTimer = new Timer();
    addRequirements(myFeeder);
  }

  @Override   // Called when the command is initially scheduled.
  public void initialize() {
    myTimer.start();
  }

  @Override   // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    if (((myShooter.getVelocity() >= ShooterConstants.targetVelocity-50) && (Math.abs(myLimelight.getX()) <= 0.8)) || (myTimer.get() > 1.70 )){
        myFeeder.runHopper(-.25);
        myFeeder.runLoader(.4);
    }
    SmartDashboard.putNumber("Shooter Timer", myTimer.get());
  }

  @Override // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    myFeeder.runBoth(0);
    myTimer.stop();
    myTimer.reset();
  }

  @Override   // Returns true when the command should end.
  public boolean isFinished() {
    return myTimer.get() >= amountOfTime;
  }
}
