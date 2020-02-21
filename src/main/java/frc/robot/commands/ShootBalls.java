/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.OIConstants.*;

public class ShootBalls extends CommandBase {

  private final Shooter myShooter;

  private final Joystick cont;

  public ShootBalls(Shooter shooter) {
    myShooter = shooter;
    cont = new Joystick(contPort);
    addRequirements(myShooter);
  }

  @Override
  public void initialize() { // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {   // Called every time the scheduler runs while the command is scheduled.
  }

  @Override
  public void end(boolean interrupted) {   // Called once the command ends or is interrupted.
  }

  @Override
  public boolean isFinished() {   // Returns true when the command should end.
    return false;
  }
}
