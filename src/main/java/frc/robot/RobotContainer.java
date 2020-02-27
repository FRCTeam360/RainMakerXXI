/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TrajectoryConstants;

//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.commands.autoCommands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

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
  private final ShootBalls shootBalls = new ShootBalls(shooter, feeder);
  private final ManualShooter manualShooter = new ManualShooter(shooter);
  private final RunFeeder runFeeder = new RunFeeder(feeder);
  private final Climb climb = new Climb(climber);

  Joystick joyR = new Joystick(OIConstants.joyRPort);
  Joystick joyOI = new Joystick(OIConstants.contPort);

  private final AlignShoot alignShoot = new AlignShoot(drivetrain, limelight, shooter, feeder, intake);
  //private final Align align = new Align(drivetrain, limelight); //This is unused currently and is called only in alignShoot

  private final Command m_autoCommand_backup = new SequentialCommandGroup(
    new ParallelRaceGroup(      
      new Align(drivetrain, limelight), 
      new AutoLoadBalls(feeder, limelight, shooter), //This one has the abort feature in it
      new AutoRunIntake(intake),
      new ShooterRamp(shooter) 
    ),
    new AutoBackupOnTicks(drivetrain)
  );

  private final RamseteCommand m_autoCommand_sanityS = new RamseteCommand(
    TrajectoryConstants.sanityS,
    drivetrain::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(
      AutoConstants.ksVolts,
      AutoConstants.kvVoltSecondsPerMeter,
      AutoConstants.kaVoltSecondsSquaredPerMeter
    ),
    AutoConstants.kDriveKinematics,
    drivetrain::getWheelSpeeds,
    new PIDController(AutoConstants.kPDriveVel, 0, 0),
    new PIDController(AutoConstants.kPDriveVel, 0, 0),
    drivetrain::tankDriveVolts,
    drivetrain
  );
  private final Command m_autoCommand_sanityLine = new SequentialCommandGroup(
    new RamseteCommand(
      TrajectoryConstants.sanityLine,
      drivetrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(AutoConstants.ksVolts,
      AutoConstants.kvVoltSecondsPerMeter,
      AutoConstants.kaVoltSecondsSquaredPerMeter),
      AutoConstants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(AutoConstants.kPDriveVel, 0, 0),
      new PIDController(AutoConstants.kPDriveVel, 0, 0),
      drivetrain::tankDriveVolts,
      drivetrain
    ),
    new RamseteCommand(
      TrajectoryConstants.sanityLineRev,
      drivetrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(AutoConstants.ksVolts,
      AutoConstants.kvVoltSecondsPerMeter,
      AutoConstants.kaVoltSecondsSquaredPerMeter),
      AutoConstants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(AutoConstants.kPDriveVel, 0, 0),
      new PIDController(AutoConstants.kPDriveVel, 0, 0),
      drivetrain::tankDriveVolts,
      drivetrain
    )
  );
  /*
  private final Command m_autoCommand_fwdRev = new SequentialCommandGroup(
    new RamseteCommand(
      TrajectoryConstants.sanityLine,
      drivetrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(AutoConstants.ksVolts,
      AutoConstants.kvVoltSecondsPerMeter,
      AutoConstants.kaVoltSecondsSquaredPerMeter),
      AutoConstants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(AutoConstants.kPDriveVel, 0, 0),
      new PIDController(AutoConstants.kPDriveVel, 0, 0),
      drivetrain::tankDriveVolts,
      drivetrain
    ),
    new RamseteCommand(
      TrajectoryConstants.sanityLineRev,
      drivetrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(AutoConstants.ksVolts,
      AutoConstants.kvVoltSecondsPerMeter,
      AutoConstants.kaVoltSecondsSquaredPerMeter),
      AutoConstants.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(AutoConstants.kPDriveVel, 0, 0),
      new PIDController(AutoConstants.kPDriveVel, 0, 0),
      drivetrain::tankDriveVolts,
      drivetrain
    )
  );
  */
  /*
  private final Command m_autoCommand_left = new SequentialCommandGroup(
    new ParallelRaceGroup(      
      new Align(drivetrain, limelight), 
      new AutoLoadBalls(feeder, limelight, shooter), //This one has the abort feature in it
      new AutoRunIntake(intake),
      new ShooterRamp(shooter) 
    ),
    new ParallelRaceGroup(
      new AutoRunIntake(intake), //Never ends
      new RamseteCommand( //Ends itself
        TrajectoryConstants.theAutoPathFirstStage,
        drivetrain::getPose, 
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter),
        AutoConstants.kDriveKinematics, 
        drivetrain::getWheelSpeeds, 
        new PIDController(AutoConstants.kPDriveVel, 0, 0), 
        new PIDController(AutoConstants.kPDriveVel, 0, 0), 
        drivetrain::tankDriveVolts,
        drivetrain
      ) 
    )
  );
  */

  private final SequentialCommandGroup m_autoCommand_middle = new SequentialCommandGroup( //shoot, backup, mid balls,shoot
    new ParallelRaceGroup(      
      new Align(drivetrain, limelight), 
      new AutoLoadBalls(feeder, limelight, shooter), //This one has the abort feature in it
      new AutoRunIntake(intake),
      new ShooterRamp(shooter) 
    ),
    new ParallelRaceGroup(
      new AutoRunIntake(intake), //Never ends
      new RamseteCommand( //Ends itself
        TrajectoryConstants.theAutoPathFirstStage, //Stage 1
        drivetrain::getPose, 
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter),
        AutoConstants.kDriveKinematics, 
        drivetrain::getWheelSpeeds, 
        new PIDController(AutoConstants.kPDriveVel, 0, 0), 
        new PIDController(AutoConstants.kPDriveVel, 0, 0), 
        drivetrain::tankDriveVolts,
        drivetrain
      ) 
    ),
    new ParallelRaceGroup(
      new AutoRunIntake(intake), //Never ends
      new RamseteCommand( //Ends itself
        TrajectoryConstants.theAutoPathSecondStage, //Stage 2
        drivetrain::getPose, 
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter),
        AutoConstants.kDriveKinematics, 
        drivetrain::getWheelSpeeds, 
        new PIDController(AutoConstants.kPDriveVel, 0, 0), 
        new PIDController(AutoConstants.kPDriveVel, 0, 0), 
        drivetrain::tankDriveVolts,
        drivetrain
      ) 
    ),
    new ParallelRaceGroup(      
      new Align(drivetrain, limelight), 
      new AutoLoadBalls(feeder, limelight, shooter), //This one has the abort feature in it
      new AutoRunIntake(intake),
      new ShooterRamp(shooter) 
    )
    
  );
  
  private final SequentialCommandGroup m_autoCommand_right = new SequentialCommandGroup( //Trench Run Auto
    new ParallelRaceGroup(      
      //new Align(drivetrain, limelight), //Disabled if it's lined up correctly
      new AutoLoadBalls(feeder, limelight, shooter),  //Ends after a time in seconds
      new AutoRunIntake(intake), 
      new ShooterRamp(shooter) 
    ),
    new ParallelRaceGroup(
      new RamseteCommand( //Ends itself
        TrajectoryConstants.trenchRunPathFirstStage, //Stage 1
        drivetrain::getPose, 
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter),
        AutoConstants.kDriveKinematics, 
        drivetrain::getWheelSpeeds, 
        new PIDController(AutoConstants.kPDriveVel, 0, 0), 
        new PIDController(AutoConstants.kPDriveVel, 0, 0), 
        drivetrain::tankDriveVolts,
        drivetrain
      ),
      new AutoRunIntake(intake) //never ends, just runs intake full bohr
    ),
    new ParallelRaceGroup(
      new RamseteCommand( //Ends itself
        TrajectoryConstants.trenchRunPathSecondStage, //Stage 2
        drivetrain::getPose, 
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter),
        AutoConstants.kDriveKinematics, 
        drivetrain::getWheelSpeeds, 
        new PIDController(AutoConstants.kPDriveVel, 0, 0), 
        new PIDController(AutoConstants.kPDriveVel, 0, 0), 
        drivetrain::tankDriveVolts,
        drivetrain
      )
      //Intake not needed for return trip
    ),
    new ParallelRaceGroup(      
      new Align(drivetrain, limelight), //Auto shoot @ the end
      new AutoLoadBalls(feeder, limelight, shooter),  //Ends after a time in seconds
      new AutoRunIntake(intake), 
      new ShooterRamp(shooter) 
    )

  );
  
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    drivetrain.setDefaultCommand(joystickTankDrive);
    pneumatics.setDefaultCommand(pressurize);
    shifter.setDefaultCommand(shift);
    limelight.setDefaultCommand(runCamera);
    feeder.setDefaultCommand(runFeeder);
    intake.setDefaultCommand(runIntake);
    climber.setDefaultCommand(climb);
    shooter.setDefaultCommand(manualShooter);

    configureButtonBindings();

    //m_chooser.addOption("fwd & rev Sanity", m_autoCommand_fwdRev);
    m_chooser.addOption("Line Sanity", m_autoCommand_sanityLine);
    m_chooser.addOption("S Sanity", m_autoCommand_sanityS);

    m_chooser.addOption("Anywhere Auto", m_autoCommand_backup); //Shoot & backup
    //m_chooser.addOption("Left Auto", m_autoCommand_left); 
    m_chooser.addOption("Middle Auto", m_autoCommand_middle); //Middle auto
    m_chooser.addOption("Trench Run Auto", m_autoCommand_right); //Right auto for the trench run

    SmartDashboard.putData("Auto Choice", m_chooser);
  }

  public DriveTrain gDriveTrain () {
    return drivetrain;
  }

  public Shifter gShifter() {
    return shifter;
  }

  private void configureButtonBindings() {
    new JoystickButton(joyR, 5).whenPressed(switchCamMode);
    new JoystickButton(joyOI, 8).whenHeld(shootBalls);
    if(limelight.validTarget()) { 
      new JoystickButton(joyR , 1).whenHeld(alignShoot);  //This whenHeld schedules a command when a trigger changes from inactive to active (or, accordingly, when a button is initially pressed) and cancels it when the trigger becomes inactive again (or the button is released). The command will not be re-scheduled if it finishes while the trigger is still active.
    }
  }
  
  public Command getAutonomousCommand() { //Called int robot autonomousInit which schedules the command sent to it
    return m_chooser.getSelected(); //Sends the selected autonomous command initialized above 
  }
}