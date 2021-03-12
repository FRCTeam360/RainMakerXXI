// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.SkillsOneGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import frc.robot.Constants.teachihngTrajectories;

import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class TeachingAutonomous extends SequentialCommandGroup {

  public TeachingAutonomous( DriveTrain drivetrain) {
      super(
        new MoveWithRamsete(
          teachihngTrajectories.rev, 
          drivetrain
        )
        .andThen(() -> drivetrain.tankDriveVolts(0,0)), 
        new MoveWithRamsete(
          teachihngTrajectories.fwd, 
          drivetrain
        )
        .andThen(() -> drivetrain.tankDriveVolts(0,0)), 
        new MoveWithRamsete(
          teachihngTrajectories.rev2, 
          drivetrain
        )
        .andThen(() -> drivetrain.tankDriveVolts(0,0))
      );
  }
}
