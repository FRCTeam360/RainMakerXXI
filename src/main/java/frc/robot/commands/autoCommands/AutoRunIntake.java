/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Intake;

public class AutoRunIntake extends CommandBase {

  private final Intake myIntake;

  public AutoRunIntake(Intake intake) {
    myIntake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    myIntake.run(.5); //Half Speed?
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myIntake.run(0.0); //Stops the intake when the command is ended
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { //Will be run as part of a parelel race and is therefore unneeded
    return false;
  }
}
