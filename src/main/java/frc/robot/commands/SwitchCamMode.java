/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Limelight;

public class SwitchCamMode extends InstantCommand {
  
  Limelight myLimelight;

  public SwitchCamMode(Limelight limelight) {
    myLimelight = limelight;
    addRequirements(myLimelight);     // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {   // Called when the command is initially scheduled.
    myLimelight.changeCamMode();
  }
}
