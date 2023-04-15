package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

import static edu.wpi.first.math.MathUtil.clamp;

public class ChassisDriveAutonLockHeading extends CommandBase {

    Chassis m_chassis;
    double fwdSpeed, strSpeed, time;

    PIDController rot_controller = new PIDController(0.01, 0, 0);

    Timer timer = new Timer();

    public ChassisDriveAutonLockHeading(Chassis chassis, double fwdSpeed, double strSpeed, double time) {
        m_chassis = chassis;
        this.fwdSpeed = fwdSpeed;
        this.strSpeed = strSpeed;
        this.time = time;

        addRequirements(m_chassis);

        rot_controller.enableContinuousInput(-180, 180);
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    m_chassis.solveAngles(fwdSpeed, strSpeed, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = -(m_chassis.ahrs.getAngle() % 360);
    m_chassis.runSwerve(fwdSpeed, strSpeed, clamp(-rot_controller.calculate(angle, 0), -0.75, 0.75));
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
