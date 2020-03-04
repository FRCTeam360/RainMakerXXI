/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;

public class AutoBackupOnTicks extends CommandBase {

  DriveTrain driveTrain;

  public AutoBackupOnTicks(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain); // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void execute() {   // Called every time the scheduler runs while the command is scheduled.
    driveTrain.driveLMAX(-.125);
    driveTrain.driveRMAX(-.125); //1/8 speed, fast enough robot moves, slow enough it goes slow
  }

  @Override
  public void end(boolean interrupted) {   // Called once the command ends or is interrupted.
    driveTrain.driveLMAX(0.0);
    driveTrain.driveRMAX(0.0);
  }

  @Override
  public boolean isFinished() {   // Returns true when the command should end.
    return driveTrain.avgMeterTrav() <= -2.0;
  }
}
