/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Limelight;

public class RunCamera extends CommandBase {

  private final Limelight myLimelight;

  public RunCamera(Limelight limelight) {
    myLimelight = limelight;
    addRequirements(myLimelight);
  }

  public boolean isFinished() { return false; }
}
