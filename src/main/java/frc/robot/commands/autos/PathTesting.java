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
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class PathTesting {

  public String trajectoryString;
  public DriveTrain drivetrain;

  public Path path;
  public Trajectory traj;
  public Trajectory transTraj;
  public Pose2d bOrigin;

  public PathTesting( String trajectoryStringInput , DriveTrain drivetrainInput ) {
    trajectoryString = trajectoryStringInput;
    drivetrain = drivetrainInput;

    try {
      path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryString);
      traj = TrajectoryUtil.fromPathweaverJson(path);
      //System.out.println("======Able to open trajectory: " + trajectoryString + "======");
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryString, ex.getStackTrace());
      System.out.println("======Unable to open trajectory: " + trajectoryString + "======");
    }

    bOrigin = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    transTraj = traj.relativeTo(bOrigin);
    System.out.println(transTraj.toString());
  }

  public SequentialCommandGroup getCommand() {
    return new MoveWithRamsete(transTraj, drivetrain).andThen(() -> drivetrain.tankDriveVolts(0,0));
  }
}
