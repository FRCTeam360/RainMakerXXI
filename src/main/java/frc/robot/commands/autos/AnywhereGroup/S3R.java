/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autos.AnywhereGroup;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class S3R extends SequentialCommandGroup { //Could be inline but done like this for simplicity
  public S3R( DriveTrain drivetrain, Limelight limelight, Feeder feeder, Shooter shooter, Intake intake ) {

    super(
      new ParallelRaceGroup(      
        new Align(drivetrain, limelight), 
        new AutoLoadBalls(feeder, limelight, shooter, 4.0), //This one has the abort feature in it
        new AutoRunIntake(intake),
        new ShooterRamp(shooter) 
      ),
      new AutoBackupOnTicks(drivetrain)
    );

  }
}
