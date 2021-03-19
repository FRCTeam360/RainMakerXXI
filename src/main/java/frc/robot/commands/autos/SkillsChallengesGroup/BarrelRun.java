// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.SkillsChallengesGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import frc.robot.Constants.BarrelRunTrajectories;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

//Skills Challenge: Barrell Racing Path
public class BarrelRun extends SequentialCommandGroup {
  public BarrelRun(DriveTrain drivetrain) {
    super(
        new MoveWithRamsete(
          BarrelRunTrajectories.first, 
          drivetrain
        )
        .andThen(() -> drivetrain.tankDriveVolts(0,0))/*,
        new ResetEncorderPosition(drivetrain),
        new MoveWithRamsete(
          BarrelRunTrajectories.backFromZero, 
          drivetrain
        )
        .andThen(() -> drivetrain.tankDriveVolts(0,0)) */
    );
  }
}
