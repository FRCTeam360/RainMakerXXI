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

    super(new parStepOne( intake , shooter ) , new parStepTwo( driveTrain , intake ) , new parStepThree( intake , limelight ) ); //Run 3 parelell parts in order

  }
}


/*
COPY FROM DOCUMENTATION:
I. drivetrain, intake, shooter, limelight
    1. intake, shooter
        B. intake
        C. shooter
    2. drivetrain, intake
        A. drivetrain
        B. intake
    3. intake, limelight
        A. intake
        B. LimeLight


*/