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
    if(Math.abs(driverCont.getY(Hand.kLeft)) >= xboxDeadzone || Math.abs(driverCont.getX(Hand.kLeft)) >= xboxDeadzone){
      if(driverCont.getX(Hand.kLeft) <= -xboxDeadzone){
        System.out.println("less than -" + xboxDeadzone);
        myDriveTrain.driveRMAX(-1 * driverCont.getX(Hand.kLeft) * 0.8);
        if(Math.abs(driverCont.getY(Hand.kLeft)) >= xboxDeadzone){
          myDriveTrain.driveLMAX(-1 * driverCont.getY(Hand.kLeft) * 0.8);
        } else {
          myDriveTrain.driveLMAX(0);
        }
      } else if(driverCont.getX(Hand.kLeft) >= xboxDeadzone) {
        System.out.println("greater than " + xboxDeadzone);
        myDriveTrain.driveLMAX(1 * driverCont.getX(Hand.kLeft) * 0.8);
        if(Math.abs(driverCont.getY(Hand.kLeft)) >= xboxDeadzone){
          myDriveTrain.driveRMAX(-1 * driverCont.getY(Hand.kLeft));
        } else {
          myDriveTrain.driveRMAX(0);
        }
      } else {
        System.out.println("forward");
        myDriveTrain.driveRMAX(-1 * driverCont.getY(Hand.kLeft) * 0.8);
        myDriveTrain.driveLMAX(-1 * driverCont.getY(Hand.kLeft) * 0.8);
      }
    } else {
      myDriveTrain.driveRMAX(0);
      myDriveTrain.driveLMAX(0);
    }
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
