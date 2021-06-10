// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.DriveTrain;

//Resets the encoder position & odometry of the robot to 0.
public class ResetEncorderPosition extends InstantCommand {

  private final DriveTrain driveTrain;

  public ResetEncorderPosition(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  @Override   // Called when the command is initially scheduled.
  public void initialize() {
    driveTrain.resetEncPos();
  }
}
