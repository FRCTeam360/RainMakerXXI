/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Intake;

public class AutoLowerIntake extends InstantCommand {

  private final Intake myIntake;

  public AutoLowerIntake(Intake intake) {
    myIntake = intake;
    addRequirements(myIntake);
  }

  @Override
  public void initialize() {
    myIntake.intakeDown();
  }
}
