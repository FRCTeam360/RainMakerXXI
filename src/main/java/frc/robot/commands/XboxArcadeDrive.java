// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.OIConstants.*;

public class XboxArcadeDrive extends CommandBase {

  private final DriveTrain myDriveTrain;

  private final XboxController driverCont;

  /** Creates a new XboxArcadeDrive. */
  public XboxArcadeDrive(DriveTrain driveTrain) {
    driverCont = new XboxController(driverContPort);

    myDriveTrain = driveTrain;

    addRequirements(myDriveTrain); // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightLeftSquared = 0;
    double upDownSquared = 0;
    double driveRight = 0;
    double driveLeft = 0;

    if(Math.abs(driverCont.getX(Hand.kLeft)) >= xboxDeadzone) {
      rightLeftSquared = driverCont.getX(Hand.kLeft) * driverCont.getX(Hand.kLeft);
      if(driverCont.getX(Hand.kLeft) < 0){
        rightLeftSquared = rightLeftSquared * -1;
      }
    }
    if(Math.abs(driverCont.getY(Hand.kLeft)) >= xboxDeadzone) {
      upDownSquared = -1 * driverCont.getY(Hand.kLeft) * driverCont.getY(Hand.kLeft);
      if(driverCont.getY(Hand.kLeft) < 0){
        upDownSquared = upDownSquared * -1;
      }
    }
    
    driveLeft = upDownSquared + rightLeftSquared;
    driveRight = upDownSquared - rightLeftSquared;

    driveLeft = Math.min(driveLeft, 1);
    driveRight = Math.min(driveRight, 1);
    driveLeft = Math.max(driveLeft, -1);
    driveRight = Math.max(driveRight, -1);
 
    // if(Math.abs(upDown) >= xboxDeadzone || Math.abs(rightLeft) >= xboxDeadzone){
    //   if(rightLeft <= -xboxDeadzone){
    //     driveRight = -1 * rightLeft;
    //     if(Math.abs(upDown) >= xboxDeadzone){
    //       driveLeft = upDown;
    //     } else {
    //       driveLeft = 1 * rightLeft;
    //     }
    //   } else if(rightLeft >= xboxDeadzone) {
    //     driveLeft = 1 * rightLeft;
    //     if(Math.abs(upDown) >= xboxDeadzone){
    //       driveRight = -1 * upDown;
    //     } else {
    //       driveRight = -1 * rightLeft;
    //     }
    //   } else {
    //     System.out.println("forward");
    //     driveRight = -1 * upDown;
    //     driveLeft = -1 * upDown;      }
    // }
    myDriveTrain.driveLMAX(driveLeft * 0.8);
    myDriveTrain.driveRMAX(driveRight * 0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
