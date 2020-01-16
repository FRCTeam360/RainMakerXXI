/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



/**
 * The VR is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // public TalonSRX shooterMaster;
  // public TalonSRX shooterSlave;

  // Joystick joy;
  // Joystick joy1;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // shooterMaster = new TalonSRX(0);
    // shooterSlave = new TalonSRX(1);
    // shooterSlave.follow(shooterMaster); //bind them together

    // joy = new Joystick(0);
    // joy1 = new Joystick(1);

    // shooterMaster.configFactoryDefault();
    // shooterSlave.configFactoryDefault();

    // shooterMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative , Constants.kPIDLoopIdx , Constants.kTimeOutMs);
    // shooterMaster.setSensorPhase(false);

    // shooterMaster.configNominalOutputForward( 0 , Constants.kTimeOutMs);
    // shooterMaster.configNominalOutputReverse( 0 , Constants.kTimeOutMs);
    // shooterMaster.configPeakOutputForward( 1 , Constants.kTimeOutMs);
    // shooterMaster.configPeakOutputReverse( -1 , Constants.kTimeOutMs);

    // shooterMaster.config_kF(Constants.kPIDLoopIdx, Constants.kF , Constants.kTimeOutMs );
    // shooterMaster.config_kP(Constants.kPIDLoopIdx, Constants.kP , Constants.kTimeOutMs );
    // shooterMaster.config_kI(Constants.kPIDLoopIdx, Constants.kI , Constants.kTimeOutMs );
    // shooterMaster.config_kD(Constants.kPIDLoopIdx, Constants.kD , Constants.kTimeOutMs );

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }


   //This function is called periodically during operator control.

  @Override
  public void teleopPeriodic() {
    
    // if ( joy.getRawButton(1) ) {
    //   shooterMaster.set(ControlMode.Velocity , (((Constants.targetRpm * 4096) / 600) / 2) ); //divided by 2 is for our gear ratio
    //   System.out.println( (( (Constants.targetRpm * 4096) / 600) / 2) + "   " );    //about= 13,650
    // } else {
    //   shooterMaster.set(ControlMode.PercentOutput , joy.getRawAxis(1) );
    // }
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}