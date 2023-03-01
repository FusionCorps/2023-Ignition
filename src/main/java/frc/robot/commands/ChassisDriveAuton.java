package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Chassis.RunSwerve;
import frc.robot.subsystems.Chassis;
import static frc.robot.RobotContainer.m_chassis;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.StrictMath.PI;

public class ChassisDriveAuton extends CommandBase {

    private SlewRateLimiter fwdLimiter = new SlewRateLimiter(6.5);
    private SlewRateLimiter strLimiter = new SlewRateLimiter(6.5);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(6.5);

    double fwdSpeed, rotSpeed, strSpeed, time;

    RunSwerve runSwerve;

    Timer timer = new Timer();

    public ChassisDriveAuton(double fwdSpeed, double rotSpeed, double strSpeed, double time) {
        this.fwdSpeed = fwdSpeed;
        this.rotSpeed = rotSpeed;
        this.strSpeed = strSpeed;
        this.time = time;

        runSwerve = new RunSwerve(fwdSpeed, rotSpeed, strSpeed);

        addRequirements(m_chassis);
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    m_chassis.solveAngles(fwdSpeed, strSpeed, rotSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      runSwerve.run();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.runSwerve(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);
  }
    

            
}
