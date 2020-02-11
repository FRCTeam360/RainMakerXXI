/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxes extends SubsystemBase {

  private static CANSparkMax spark1;
  private static CANSparkMax spark2;
  private static CANSparkMax spark3;
  private static CANSparkMax spark4;
  private static CANSparkMax spark5;
  private static CANSparkMax spark6;

  public SparkMaxes() {
    spark1 = new CANSparkMax(1 , MotorType.kBrushless);
    spark2 = new CANSparkMax(2 , MotorType.kBrushless);
    spark3 = new CANSparkMax(3 , MotorType.kBrushless);
    spark4 = new CANSparkMax(4 , MotorType.kBrushless);
    spark5 = new CANSparkMax(5 , MotorType.kBrushless);
    spark6 = new CANSparkMax(6 , MotorType.kBrushless);
  }

  public void runPercent (double pPower) {
    spark1.set(pPower);
    spark2.set(pPower);
    spark3.set(pPower);
    spark4.set(pPower);
    spark5.set(pPower);
    spark6.set(pPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
