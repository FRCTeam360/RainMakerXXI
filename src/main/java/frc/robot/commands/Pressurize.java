/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class Pressurize extends CommandBase {

  private Timer timer;
  private boolean shouldRun;

  private final Pneumatics pneumatics;

  public Pressurize( Pneumatics inPneumatics ) {
    pneumatics = inPneumatics;
    timer = new Timer();
	  shouldRun = true;
	  timer.reset();
    timer.stop();
    
    addRequirements(inPneumatics); // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shouldRun == true && RobotController.getInputVoltage() > 10) {
      pneumatics.pressurize();
    } else if (shouldRun == true && ! (RobotController.getInputVoltage() > 10)) {
      shouldRun = false;
      pneumatics.stop();
      timer.reset();
      timer.start();
    }
    if (timer.get() > 0.5) {
      timer.reset();
      timer.stop();
      shouldRun = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
	  timer.reset();
	  pneumatics.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
