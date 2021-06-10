/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

import frc.robot.Constants.AutoConstants;

//WHEN CALLING THIS METHOD: MoveWithRamsete(trajectory, drivetrain).andThen(() -> drivetrain.tankDriveVolts(0,0));

public class MoveWithRamsete extends RamseteCommand{

    public MoveWithRamsete (Trajectory traj, DriveTrain drivetrain) {
        super(
            traj, //Input send trajectory
            drivetrain::getPose, 
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter),
            AutoConstants.kDriveKinematics, 
            drivetrain::getWheelSpeeds, 
            new PIDController(AutoConstants.kPDriveVel, 0, 0), 
            new PIDController(AutoConstants.kPDriveVel, 0, 0), 
            drivetrain::tankDriveVolts,
            drivetrain
        );
    }

}
