/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Navx;

public class NavReadout extends CommandBase {
  Navx navx;
  public NavReadout(Navx navx) {
    this.navx = navx;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(navx);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}