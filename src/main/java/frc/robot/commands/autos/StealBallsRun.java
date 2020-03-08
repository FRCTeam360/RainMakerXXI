/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import frc.robot.Constants.stealBallsTrajectories;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class StealBallsRun extends SequentialCommandGroup {
  public StealBallsRun( DriveTrain drivetrain, Limelight limelight, Feeder feeder, Shooter shooter, Intake intake ) {
    super(
      new ParallelRaceGroup( //Run path and intake
        new MoveWithRamsete(
          stealBallsTrajectories.stageOne, //Ends when path is complete
          drivetrain
        )
        .andThen(() -> drivetrain.tankDriveVolts(0,0)),
        new AutoRunIntake(intake)
      ),
      new ParallelRaceGroup(
        new MoveWithRamsete(
          stealBallsTrajectories.stageTwo, 
          drivetrain
        )
        .andThen(() -> drivetrain.tankDriveVolts(0,0)),
        new ShooterRamp(shooter)
      ),
      new ParallelRaceGroup(    //Align shoot
        new Align(drivetrain, limelight), 
        new AutoLoadBalls(feeder, limelight, shooter, 8.0), //4.5 seconds total
        new AutoRunIntake(intake),
        new ShooterRamp(shooter) 
      )
      
    );

  }
}

