/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

import static frc.robot.Constants.LimelightConstants.*;
import static frc.robot.Constants.DriveTrainConstants.*;

public class Align extends CommandBase {

  private DriveTrain myDriveTrain;
  private Limelight myLimelight;
  
  private double steeringAdjust;
  private double aimError;

  private Timer timer;

  public Align(DriveTrain driveTrain, Limelight limelight) {
    myDriveTrain = driveTrain;
    myLimelight = limelight;

    steeringAdjust = 0;
    aimError = 0;

    timer = new Timer();
    timer.stop();
	  timer.reset();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myDriveTrain, myLimelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    aimError = myLimelight.getX() / 29.8;
    steeringAdjust = steer * aimError;
    if (myLimelight.getX() > .2) {
      steeringAdjust += AimMinCmd;
      timer.stop();
      timer.reset();
    } else if (myLimelight.getX() < -.2){ 
      steeringAdjust -= AimMinCmd;
      timer.stop();
      timer.reset();
    } else {
      timer.start();
    }
    myDriveTrain.driveLMAX(steeringAdjust);
    myDriveTrain.driveRMAX(-steeringAdjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myDriveTrain.driveLMAX(0);
    myDriveTrain.driveRMAX(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= .1 && Math.abs(myLimelight.getX()) < .4;
  }
}
