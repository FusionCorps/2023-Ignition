package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.math.Tilt;
import frc.robot.subsystems.Chassis;
import static frc.robot.RobotContainer.m_chassis;

public class ChassisAltAutoBalance extends CommandBase {

    /* Here's how this auto balance works:
     * It uses a custom Proportional-only PID controller that subtracts the current robot angle from the target angle (basically negates the current angle
     *   because the target angle is zero).
     * It plugs that value into the chassis to make it drive.
     * If the angle is within a certain margin of error of the target angle, a stopwatch starts.
     * If the robot stays within the margin of error for x number of seconds, the command terminates.*/

    Timer endTimer;

    private double maxSpeed = .1;
    private double error;
    private double drivePower;
    private double currentAngle;

    boolean inMarginofError;

    boolean timerStarted;

    RunSwerve runSwerve;

    public ChassisAltAutoBalance(Chassis chassis){
        runSwerve = new RunSwerve(0, 0, 0);
        endTimer = new Timer();

        addRequirements(m_chassis);
    }

    @Override
    public void initialize(){
        inMarginofError = false;
    }

    @Override
    public void execute(){
        double yaw = m_chassis.ahrs.getYaw();
        double pitch = m_chassis.ahrs.getPitch();
        double roll = m_chassis.ahrs.getRoll();
        currentAngle = Tilt.calculate(yaw, pitch, roll);

        System.out.println(currentAngle);

        error = -currentAngle +  11.75;
        drivePower = Math.min(error* Constants.AUTON_DRIVE_kP,1);

        //System.out.println(error + ", " + drivePower + ", " + currentAngle);


        if (Math.abs(drivePower) > maxSpeed) {
            drivePower = Math.copySign(maxSpeed, drivePower);
        }

        runSwerve.changeSpeeds(drivePower, 0, 0);

        runSwerve.run();

    }

    @Override
    public void end(boolean interrupted){
        m_chassis.crossWheels();
    }

    @Override
    public boolean isFinished(){
        boolean finished = false;
        inMarginofError = Math.abs(error) < Constants.CHARGE_STATION_BALANCE_ANGLE_GOAL;
        if(inMarginofError && !timerStarted){
            endTimer.start();
            timerStarted = true;
        } else if (!inMarginofError && timerStarted) {
            endTimer.stop();
            endTimer.reset();
            timerStarted = false;
        } else if (inMarginofError && endTimer.hasElapsed(Constants.CHARGE_STATION_STABILIZE_SECONDS)) {
            endTimer.stop();
            endTimer.reset();
            m_chassis.crossWheels();
            finished = true;
        }

        return finished;
    }

}