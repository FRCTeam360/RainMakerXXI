/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.OIConstants.*;

public class JoystickTankDrive extends CommandBase {

  private final DriveTrain myDriveTrain;

  private final Joystick joyR;
  private final Joystick joyL;


  /**
   * Creates a new JoystickTankDrive.
   */
  public JoystickTankDrive(DriveTrain driveTrain) {

    joyR = new Joystick(joyRPort);
    joyL = new Joystick(joyLPort);

    myDriveTrain = driveTrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(joyR.getRawAxis(1)) >= .15){
    	myDriveTrain.driveRMAX(-1 * joyR.getRawAxis(1) * 0.8);
    }else{
    	myDriveTrain.driveRMAX(0);
    }
    if(Math.abs(joyL.getRawAxis(1)) >= .15){
      myDriveTrain.driveLMAX(-1 * joyL.getRawAxis(1) * 0.8);
    }else{
    	myDriveTrain.driveLMAX(0);
    }
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
