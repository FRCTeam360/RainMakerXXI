/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;

public class AutoMoveOnTicks extends CommandBase {

  private DriveTrain driveTrain;
  private double targDist;
  private boolean direct; //True if forward, False if reverse

  public AutoMoveOnTicks(DriveTrain iDriveTrain, double iTargDist) {
    driveTrain = iDriveTrain;
    targDist = iTargDist;

    if (targDist >= 0.0) {
      direct = true; //If distance is zero or any positive value
    } else {
      direct = false; //If distance is any negative value
    }

    addRequirements(driveTrain);
  }

  @Override   // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    if (direct == true) { // if drive forward
      driveTrain.driveLMAX(.125);
      driveTrain.driveRMAX(.125); 
    } else { //If drive reverse
      driveTrain.driveLMAX(-.125);
      driveTrain.driveRMAX(-.125);
    }
  }

  @Override   // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    driveTrain.driveLMAX(0.0); //Stops both sides of the driveTrain
    driveTrain.driveRMAX(0.0);
  }

  @Override   // Returns true when the command should end.
  public boolean isFinished() {
    if (direct == true) { //if drive fwd
      return driveTrain.avgMeterTrav() >= targDist;
    } else { //If drive reverse
      return driveTrain.avgMeterTrav() <= targDist;
    }
  }
}
