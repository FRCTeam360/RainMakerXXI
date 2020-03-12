/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autos.CenterGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import frc.robot.Constants.middleRunTrajectories;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class cS3Gr5Sm extends SequentialCommandGroup {
  public cS3Gr5Sm( DriveTrain drivetrain, Limelight limelight, Feeder feeder, Shooter shooter, Intake intake ) {
    super(
      new ParallelRaceGroup(    //Align shoot
        new Align(drivetrain, limelight), 
        new AutoLoadBalls(feeder, limelight, shooter, 4.0), //4.5 seconds total
        new AutoRunIntake(intake),
        new ShooterRamp(shooter) 
      ),
      new ParallelRaceGroup( //Run path and intake
        new MoveWithRamsete(
          middleRunTrajectories.stageOne, //Ends when path is complete
          drivetrain
        )
        .andThen(() -> drivetrain.tankDriveVolts(0,0)),
        new AutoRunIntake(intake)
      ),
      new ParallelRaceGroup( //Run path and intake
        new MoveWithRamsete(
          middleRunTrajectories.stagetwo, //Ends when path is complete
          drivetrain
        )
        .andThen(() -> drivetrain.tankDriveVolts(0,0)),
        new AutoRunIntake(intake)
      ),
      new ParallelRaceGroup(    //Align shoot
        new Align(drivetrain, limelight), 
        new AutoLoadBalls(feeder, limelight, shooter, 4.0), //4.5 seconds total
        new AutoRunIntake(intake),
        new ShooterRamp(shooter) 
      )
    );

  }
}
