package frc.robot.commands.chassis;

import edu.wpi.first.math.filter.SlewRateLimiter;
import static frc.robot.RobotContainer.m_chassis;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.StrictMath.PI;

public class RunSwerve {
    private SlewRateLimiter fwdLimiter = new SlewRateLimiter(6.5);
    private SlewRateLimiter strLimiter = new SlewRateLimiter(6.5);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(6.5);

    private double fwdSpeed;
    private double strSpeed;
    private double rotSpeed;

    public RunSwerve(double fwd, double str, double rot) {
        fwdSpeed = fwd;
        strSpeed = str;
        rotSpeed = rot;
    }
    public void changeSpeeds(double fwd, double str, double rot) {
        fwdSpeed = fwd;
        strSpeed = str;
        rotSpeed = rot;
    }
    public void run() {
        double angle = -(m_chassis.ahrs.getAngle() % 360);

        angle -= 40*rotSpeed;

        m_chassis.runSwerve(fwdLimiter.calculate(fwdSpeed*cos(angle/360*(2*PI)) + strSpeed*sin(angle/360*(2*PI))),
                strLimiter.calculate(fwdSpeed*sin(angle/360*(2*PI)) - strSpeed*cos(angle/360*(2*PI))),
                rotLimiter.calculate(rotSpeed));
    }
}
