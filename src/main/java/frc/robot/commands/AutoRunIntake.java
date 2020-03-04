/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Intake;

public class AutoRunIntake extends CommandBase {

  private final Intake myIntake;

  public AutoRunIntake(Intake intake) {
    myIntake = intake;
    addRequirements(intake);     // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {   // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {   // Called every time the scheduler runs while the command is scheduled.
    myIntake.run(1); //full sped
  }

  @Override
  public void end(boolean interrupted) {   // Called once the command ends or is interrupted.
    myIntake.run(0.0); //Stops the intake when the command is ended
  }


  @Override   
  public boolean isFinished() {  // Returns true when the command should end.
    return false; //Will be run as part of a parelel race and is therefore unneeded
  }
}
