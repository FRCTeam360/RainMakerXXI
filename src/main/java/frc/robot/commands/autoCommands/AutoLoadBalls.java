


package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoLoadBalls extends CommandBase { //Used by AlignShoot command

  Feeder myFeeder;
  Limelight myLimelight;
  Shooter myShooter;
  Timer myTimer;

  //feeder, limelight, shooter, intake
  public AutoLoadBalls(Feeder inFeeder, Limelight limelight, Shooter shooter) {
    myFeeder = inFeeder;
    myLimelight = limelight;
    myShooter = shooter;
    myTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    myTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((myShooter.getVelocity() >= ShooterConstants.targetVelocity - 200) && (Math.abs(myLimelight.getX()) <= 0.8)) {
      //myTimer.start();
      //if(myTimer.get() >= .25) {
        myFeeder.runHopper(.5);
        myFeeder.runLoader(.4);
      //}
    } else {
      //myFeeder.runBoth(0);
      //myTimer.stop();
      //myTimer.reset();
    }
    SmartDashboard.putNumber("Shooter Timer", myTimer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myFeeder.runLoader(0);
    myTimer.stop();
    myTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return myTimer.get() >= 7;
  }
}
