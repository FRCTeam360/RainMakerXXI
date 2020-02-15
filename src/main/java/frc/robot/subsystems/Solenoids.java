/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Solenoids extends SubsystemBase {

  private static DoubleSolenoid sol1;
  private static DoubleSolenoid sol2;
  private static DoubleSolenoid sol3;

  public Solenoids() {
    sol1 = new DoubleSolenoid( 1 , 0 );
    sol2 = new DoubleSolenoid( 3 , 2 );
    sol3 = new DoubleSolenoid( 5 , 4 );
  }

  public  void  upS1 () { sol1.set(kForward); }
  public  void downS1 () { sol1.set(kReverse); }
  public  void  upS2 () { sol2.set(kForward); }
  public  void downS2 () { sol2.set(kReverse); }
  public  void  upS3 () { sol3.set(kForward); }
  public  void downS3 () { sol3.set(kReverse); }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
