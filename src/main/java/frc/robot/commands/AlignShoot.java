/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AlignShoot extends CommandBase {

  private DriveTrain myDriveTrain;
  private Intake myIntake;
  private Shooter myShooter;
  private Feeder myFeeder;

  /**
   * Creates a new AlignShoot.
   */
  public AlignShoot(DriveTrain driveTrain, Intake intake, Shooter shooter, Feeder feeder) {
    myDriveTrain = driveTrain;
    myIntake = intake;
    myShooter = shooter;
    myFeeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myDriveTrain, myIntake, myShooter, myFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
