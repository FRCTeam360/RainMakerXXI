// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.OIConstants.*;

public class JoystickArcadeDrive extends CommandBase {
  
  private final DriveTrain myDriveTrain;

  private final Joystick joyR;
  private final Joystick joyL;

  public JoystickArcadeDrive(DriveTrain driveTrain) {
    joyR = new Joystick(joyRPort);
    joyL = new Joystick(joyLPort);

    myDriveTrain = driveTrain;

    addRequirements(myDriveTrain); // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() { // Called when the command is initially scheduled.

  }

  @Override
  public void execute() { // Called every time the scheduler runs while the command is scheduled.
    if(Math.abs(joyR.getRawAxis(1)) >= .125 || Math.abs(joyL.getRawAxis(0)) >= .125){
      if(joyL.getRawAxis(0) <= -0.125){
        System.out.println("less than -0.125");
        myDriveTrain.driveRMAX(-1 * joyL.getRawAxis(0) * 0.8);
        if(Math.abs(joyR.getRawAxis(1)) >= .125){
          myDriveTrain.driveLMAX(-1 * joyR.getRawAxis(1) * 0.8);
        } else {
          myDriveTrain.driveLMAX(0);
        }
      } else if(joyL.getRawAxis(0) >= 0.125) {
        System.out.println("greater than 0.125");
        myDriveTrain.driveLMAX(1 * joyL.getRawAxis(0) * 0.8);
        if(Math.abs(joyR.getRawAxis(1)) >= .125){
          myDriveTrain.driveRMAX(-1 * joyR.getRawAxis(1) * 0.8);
        } else {
          myDriveTrain.driveRMAX(0);
        }
      } else {
        System.out.println("forward");
        myDriveTrain.driveRMAX(-1 * joyR.getRawAxis(1) * 0.8);
        myDriveTrain.driveLMAX(-1 * joyR.getRawAxis(1) * 0.8);
      }
    } else {
      myDriveTrain.driveRMAX(0);
      myDriveTrain.driveLMAX(0);
    }
  }

  @Override
  public void end(boolean interrupted) { // Called once the command ends or is interrupted.

  }

  @Override
  public boolean isFinished() { // Returns true when the command should end.
    return false;
  }
}
