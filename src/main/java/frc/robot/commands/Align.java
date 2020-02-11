/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

import static frc.robot.Constants.LimelightConstants.*;
import static frc.robot.Constants.DriveTrainConstants.maxRPM;

public class Align extends CommandBase {

  private DriveTrain myDriveTrain;
  private Limelight mylimelight;

  private double aimError;
  private double steeringAdjust;

  /**
   * Creates a new Align.
   */
  public Align(DriveTrain driveTrain, Limelight limelight) {
    myDriveTrain = driveTrain;
    mylimelight = limelight;

    aimError = 0;
    steeringAdjust = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myDriveTrain, mylimelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    aimError = mylimelight.getX() / 29.8;
    steeringAdjust = KpAim * aimError;
    if (mylimelight.getX() > .2) steeringAdjust += AimMinCmd;
    else if (mylimelight.getX() < -.2) steeringAdjust -= AimMinCmd;
    myDriveTrain.velocityDrive(steeringAdjust * maxRPM);
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
    if ( Math.abs(aimError) <= 1 ) { //If within 1 limelight degree
      return true; //Finish
    } else {
      return false; //dont finish 
    }
  }
}
