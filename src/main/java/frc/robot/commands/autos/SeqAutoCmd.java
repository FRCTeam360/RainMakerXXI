/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autos;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SeqAutoCmd extends SequentialCommandGroup {
  public SeqAutoCmd( DriveTrain driveTrain , Intake intake , Shooter shooter , Limelight limelight ) { //Needs - drivetrain, intake, shooter, limelight

    super(new parStepOne() , new parStepTwo() , new parStepThree() ); //Run 3 parelell parts in order

  }
}
