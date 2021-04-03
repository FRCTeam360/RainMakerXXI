/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

//import frc.robot.commands.AutoRunIntake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;
//import frc.robot.subsystems.Intake;

//AlignShoot (Comments on lines 24, 25, & 26 should not be taken seriously)
public class AlignShoot extends ParallelCommandGroup {
  public AlignShoot(DriveTrain driveTrain, Limelight limelight, Shooter shooter, Feeder feeder) {
    super(
      new Align(driveTrain, limelight), //ends when aligned
      new LoadBalls(feeder, limelight, shooter), //ends on timer
      new ShooterRamp(shooter) //Ends on abort button
    );
  }
}
