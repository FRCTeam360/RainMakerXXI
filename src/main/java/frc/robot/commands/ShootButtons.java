/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;

public class ShootButtons extends CommandBase {

  private final Shooter shooter;
  private final Joystick cont;

  public ShootButtons(Shooter shooter) {
    this.shooter = shooter;
    cont = new Joystick(contPort);
    addRequirements(shooter);// Use addRequirements() here to declare subsystem dependencies.
  }

  @Override   // Called when the command is initially scheduled.
  public void initialize() {
    updateShooter();
  }
  public void updateShooter(){
    if (cont.getRawButton( 2 )) { //If A button held
      targetVelocity = 12000.0;
      SmartDashboard.putString("Button", "A");
    } else if (cont.getRawButton(3)){
      targetVelocity = 13000.0;
      SmartDashboard.putString("Button", "B");
    } else if (cont.getRawButton(4)){
      targetVelocity = 14000.0;
      SmartDashboard.putString("Button", "Y");
    } else {
      targetVelocity = backupTargetVelocity;
      SmartDashboard.putBoolean("A Button", false);
    }
  }

  @Override   // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    shooter.run();
  }

  @Override   // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    targetVelocity = backupTargetVelocity;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
