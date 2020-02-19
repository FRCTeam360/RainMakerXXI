/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
  /**
   * Creates a new RunIntake.
   */

  private final Intake myIntake;
  private final Joystick joyOI;

  public RunIntake(Intake intake) {
    joyOI = new Joystick(contPort);
    myIntake = intake;
    addRequirements(myIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joyOI.getRawButton(6)){
      myIntake.run(1);
    }else{
      myIntake.run(0);
    }
    /* //Disables the pneumatic
    if (joyOI.getRawButton(5)){
      myIntake.intakeUp();
    }else if (joyOI.getRawButton(6)){
      myIntake.intakeDown();
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
