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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AlignShoot extends ParallelCommandGroup {
  /**
   * Creates a new AlignShoot.
   */
  public AlignShoot(DriveTrain driveTrain, Limelight limelight, Shooter shooter, Feeder feeder) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new Align(driveTrain, limelight), //ends when aligned
      new LoadBalls(feeder, limelight, shooter), //ends on timer
      new ShooterRamp(shooter) //Ends on abort button
    );
  }
}
