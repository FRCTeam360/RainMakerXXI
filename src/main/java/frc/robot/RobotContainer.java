/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick; 
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj.buttons.POVButton; //up is 0, right is 90, down is 180, left is 270, and if its unpressed, its -1, use 45 intervals

import frc.robot.Constants.OIConstants;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

public class RobotContainer {   // The robot's subsystems and commands are defined here...
  public final DriveTrain drivetrain = new DriveTrain();
  public final Pneumatics pneumatics = new Pneumatics();
  public final Shifter shifter = new Shifter();
  public final Shooter shooter = new Shooter();
  public final Limelight limelight = new Limelight();
  public final Intake intake = new Intake();
  public final Feeder feeder = new Feeder();
  public final Climber climber = new Climber();

  //private final JoystickTankDrive joystickTankDrive = new JoystickTankDrive(drivetrain);
  //private final JoystickArcadeDrive joystickArcadeDrive = new JoystickArcadeDrive(drivetrain);
  private final XboxArcadeDrive xboxArcadeDrive = new XboxArcadeDrive(drivetrain);
  private final Pressurize pressurize = new Pressurize(pneumatics);
  private final Shift shift = new Shift(shifter);
  private final RunIntake runIntake = new RunIntake(intake);
  private final RunCamera runCamera = new RunCamera(limelight);
  private final SwitchCamMode switchCamMode = new SwitchCamMode(limelight);
  //private final ShootBalls shootBalls = new ShootBalls(shooter, feeder, intake);
  private final ManualShooter manualShooter = new ManualShooter(shooter);
  private final RunFeeder runFeeder = new RunFeeder(feeder);
  private final Climb climb = new Climb(climber);
  private final AlignShoot alignShoot = new AlignShoot(drivetrain, limelight, shooter, feeder);
  private final ShooterRamp shooterRamp = new ShooterRamp(shooter);

  private Joystick joyR = new Joystick(OIConstants.joyRPort);
  private Joystick joyL = new Joystick(OIConstants.joyLPort);
  private Joystick joyOI = new Joystick(OIConstants.contPort);

  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindings();
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(xboxArcadeDrive);
    pneumatics.setDefaultCommand(pressurize);
    shifter.setDefaultCommand(shift);
    limelight.setDefaultCommand(runCamera);
    feeder.setDefaultCommand(runFeeder);
    intake.setDefaultCommand(runIntake);
    climber.setDefaultCommand(climb);
    shooter.setDefaultCommand(manualShooter);
  }
  private void configureButtonBindings() {
    //new POVButton(joyOI, 0).whenPressed(intakeUp);
    //new POVButton(joyOI, 45).whenPressed(intakeUp);
    //new POVButton(joyOI, 315).whenPressed(intakeUp);
    //new POVButton(joyOI, 180).whenPressed(intakeDown);
    //new POVButton(joyOI, 135).whenPressed(intakeDown);
    //new POVButton(joyOI, 225).whenPressed(intakeDown);
    new JoystickButton(joyR, 5).whenPressed(switchCamMode);
    new JoystickButton(joyL, 1).whenHeld( shooterRamp );
    new JoystickButton(joyOI, 8).whenHeld(shooterRamp);
    new JoystickButton(joyR , 1).whenHeld(alignShoot);  //This whenHeld schedules a command when a trigger changes from inactive to active (or, accordingly, when a button is initially pressed) and cancels it when the trigger becomes inactive again (or the button is released). The command will not be re-scheduled if it finishes while the trigger is still active.
  }

}