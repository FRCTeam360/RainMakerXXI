/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Shooter extends SubsystemBase {

  private TalonSRX shooterMaster;
  private TalonSRX shooterSlave;

  private static double targetRPM;

  public Shooter() {
    shooterMaster = new TalonSRX(shooterMasterId);
    shooterSlave = new TalonSRX(shooterSlaveId);

    shooterMaster.configFactoryDefault();
    shooterSlave.configFactoryDefault();

    shooterSlave.follow(shooterMaster);

    shooterMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative , kPIDLoopIdx , kTimeOutMs);

    shooterMaster.setInverted(false);
    shooterSlave.setInverted(InvertType.FollowMaster);

    shooterMaster.setSensorPhase(false);

    shooterMaster.configNominalOutputForward( 0 , kTimeOutMs);
    shooterMaster.configNominalOutputReverse( 0 , kTimeOutMs);
    shooterMaster.configPeakOutputForward( 1 , kTimeOutMs);
    shooterMaster.configPeakOutputReverse( -1 , kTimeOutMs);

    shooterMaster.config_kF(kPIDLoopIdx, kF , kTimeOutMs );
    shooterMaster.config_kP(kPIDLoopIdx, kP , kTimeOutMs );
    shooterMaster.config_kI(kPIDLoopIdx, kI , kTimeOutMs );
    shooterMaster.config_kD(kPIDLoopIdx, kD , kTimeOutMs );
  }

  public double getVelocity() {
    return shooterMaster.getSelectedSensorVelocity(0);
  }

  public void run () {
    shooterMaster.set(ControlMode.Velocity, targetVelocity); //15900 native units is 60%
  }

  public void runWithJoy (double output) {
    shooterMaster.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", ((shooterMaster.getSelectedSensorVelocity(0) * 600.0) / 4096.0 / 2.0)); // (<velocity> * 2 * 600) / 4096 converts native units to RPM
    SmartDashboard.putNumber("Target RPM", targetRPM);
    SmartDashboard.putNumber("Total Shooter Current Draw", shooterMaster.getStatorCurrent() + shooterSlave.getStatorCurrent()); //amps of both motors driving shooter
    SmartDashboard.putNumber("Shooter Velocity", shooterMaster.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Shooter Error", shooterMaster.getClosedLoopError());
  }
}
