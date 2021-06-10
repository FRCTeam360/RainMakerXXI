// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.SkillsChallengesGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import frc.robot.Constants.SlalomRunTrajectories;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

//Skills Challenge: Slalom Racing Path
public class SlalomRun extends SequentialCommandGroup {
  public SlalomRun(DriveTrain drivetrain) {
    super(
        new MoveWithRamsete(
          SlalomRunTrajectories.slalomPath, 
          drivetrain
        )
        .andThen(() -> drivetrain.tankDriveVolts(0,0))
    );
  }
}
