/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.commands.autos.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {   // The robot's subsystems and commands are defined here...
  private final DriveTrain drivetrain = new DriveTrain();
  private final Pneumatics pneumatics = new Pneumatics();
  private final Shifter shifter = new Shifter();
  private final Shooter shooter = new Shooter();
  private final Limelight limelight = new Limelight();
  private final Intake intake = new Intake();
  private final Feeder feeder = new Feeder();
  private final Climber climber = new Climber();

  private final JoystickTankDrive joystickTankDrive = new JoystickTankDrive(drivetrain);
  private final Pressurize pressurize = new Pressurize(pneumatics);
  private final Shift shift = new Shift(shifter);
  private final RunIntake runIntake = new RunIntake(intake);
  private final RunCamera runCamera = new RunCamera(limelight);
  private final SwitchCamMode switchCamMode = new SwitchCamMode(limelight);
  private final ShootBalls shootBalls = new ShootBalls(shooter, feeder, intake);
  private final ManualShooter manualShooter = new ManualShooter(shooter);
  private final RunFeeder runFeeder = new RunFeeder(feeder);
  private final Climb climb = new Climb(climber);
  private final AlignShoot alignShoot = new AlignShoot(drivetrain, limelight, shooter, feeder, intake);
  private final ShooterRamp shooterRamp = new ShooterRamp(shooter);

  private Joystick joyR = new Joystick(OIConstants.joyRPort);
  private Joystick joyL = new Joystick(OIConstants.joyLPort);
  private Joystick joyOI = new Joystick(OIConstants.contPort);

  private final Command m_autoCommand_backup = new ThreeBallsAndLine( drivetrain, limelight, feeder, shooter, intake );
  private final Command m_autoCommand_pathTesting = new PathTesting("straightLine.wpilib.json", drivetrain).getCommand();
  private final Command m_autoCommand_trenchRun = new FullTrenchRun( drivetrain, limelight, feeder, shooter, intake );
  
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindings();
    configureAutonomousChooser();
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(joystickTankDrive);
    pneumatics.setDefaultCommand(pressurize);
    shifter.setDefaultCommand(shift);
    limelight.setDefaultCommand(runCamera);
    feeder.setDefaultCommand(runFeeder);
    intake.setDefaultCommand(runIntake);
    climber.setDefaultCommand(climb);
    shooter.setDefaultCommand(manualShooter);
  }
  private void configureButtonBindings() {
    new JoystickButton(joyR, 5).whenPressed(switchCamMode);
    new JoystickButton(joyOI, 8).whenHeld(shootBalls);
    new JoystickButton(joyL, 1).whenHeld( shooterRamp );
    new JoystickButton(joyR , 1).whenHeld(alignShoot);  //This whenHeld schedules a command when a trigger changes from inactive to active (or, accordingly, when a button is initially pressed) and cancels it when the trigger becomes inactive again (or the button is released). The command will not be re-scheduled if it finishes while the trigger is still active.
  }
  private void configureAutonomousChooser() {
    m_chooser.setDefaultOption("Anywhere Auto: Works", m_autoCommand_backup); //Makes backup auto the defaulr
    m_chooser.addOption("Path Testing: Testing", m_autoCommand_pathTesting);
    m_chooser.addOption("Trench Run Split: Testing", m_autoCommand_trenchRun);
    m_chooser.addOption("Trench Run Full: DNE", m_autoCommand_backup);
    m_chooser.addOption("Trench Steal: DNE", m_autoCommand_backup);
    m_chooser.addOption("Middle Auto: DNE", m_autoCommand_backup);
		//m_chooser.addOption(name, object);
    SmartDashboard.putData("Auto Choice", m_chooser);
  }

  public DriveTrain gDriveTrain () { return drivetrain; }
  public Shifter gShifter() { return shifter; }

  public Command getAutonomousCommand() { //Called int robot autonomousInit which schedules the command sent to it
    return m_chooser.getSelected(); //Sends the selected autonomous command initialized above 
  }
}