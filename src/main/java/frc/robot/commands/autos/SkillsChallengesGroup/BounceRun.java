// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.SkillsChallengesGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.BounceRunTrajectories;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

//
public class BounceRun extends SequentialCommandGroup {
  public BounceRun(DriveTrain drivetrain) {
    super(
        new MoveWithRamsete(
          BounceRunTrajectories.stageOne, 
          drivetrain
        ),
        //.andThen(() -> drivetrain.tankDriveVolts(0,0)), 
        new MoveWithRamsete(
          BounceRunTrajectories.stageTwo, 
          drivetrain
        ),
        //.andThen(() -> drivetrain.tankDriveVolts(0,0)), 
        new MoveWithRamsete(
          BounceRunTrajectories.stageThree, 
          drivetrain
        ),
        //.andThen(() -> drivetrain.tankDriveVolts(0,0)), 
        new MoveWithRamsete(
          BounceRunTrajectories.stageFour, 
          drivetrain
        )
        .andThen(() -> drivetrain.tankDriveVolts(0,0))
    );
  }
}
