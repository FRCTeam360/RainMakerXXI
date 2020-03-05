/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autos;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class PathTesting extends InstantCommand {

  public String sCurveTrajectoryString;
  public DriveTrain drivetrain;

  public Path path;
  public Trajectory traj;

  public PathTesting( String sCurveTrajectoryStringInput , DriveTrain drivetrainInput ) {

    sCurveTrajectoryString = sCurveTrajectoryStringInput;
    drivetrain = drivetrainInput;

    try {
      path = Filesystem.getDeployDirectory().toPath().resolve(sCurveTrajectoryString);
      traj = TrajectoryUtil.fromPathweaverJson(path);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + sCurveTrajectoryString, ex.getStackTrace());
    }
    
    addRequirements(drivetrain);
  }

  @Override   // Called when the command is initially scheduled.
  public void initialize() {
    new MoveWithRamsete(traj, drivetrain).andThen(() -> drivetrain.tankDriveVolts(0,0));
  }
}
