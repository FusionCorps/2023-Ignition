package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisDriveAuton extends CommandBase {
    
    Chassis m_chassis;
    double fwdSpeed, rotSpeed, strSpeed, time;

    Timer timer = new Timer();

    public ChassisDriveAuton(Chassis chassis, double fwdSpeed, double rotSpeed, double strSpeed, double time) {
        m_chassis = chassis;
        this.fwdSpeed = fwdSpeed;
        this.rotSpeed = rotSpeed;
        this.strSpeed = strSpeed;
        this.time = time;

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
    m_chassis.runSwerve(fwdSpeed, strSpeed, rotSpeed);
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
