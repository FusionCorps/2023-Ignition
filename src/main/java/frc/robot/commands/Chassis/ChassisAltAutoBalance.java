package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class ChassisAltAutoBalance extends CommandBase {

    /* Here's how this auto balance works:
    * It uses a custom Proportional-only PID controller that subtracts the current robot angle from the target angle (basically negates the current angle
    *   because the target angle is zero).
    * It plugs that value into the chassis to make it drive.
    * If the angle is within a certain margin of error of the target angle, a stopwatch starts.
    * If the robot stays within the margin of error for x number of seconds, the command terminates.*/
    Chassis mChassis;

    Timer endTimer;

    private double error;
    private double drivePower;
    private double currentAngle;

    boolean inMarginofError;

    boolean timerStarted;

    public ChassisAltAutoBalance(Chassis chassis){
        mChassis = chassis;
        endTimer = new Timer();
        
        addRequirements(mChassis);
    }

    @Override
    public void initialize(){
        inMarginofError = false;
    }

    @Override
    public void execute(){
        currentAngle = mChassis.getRoll();

        error = -currentAngle + 11.75;
        drivePower = Math.min(error* Constants.AUTON_DRIVE_kP,1);

        System.out.println(error + ", " + drivePower + ", " + currentAngle);


        if (Math.abs(drivePower) > 0.1) {
            drivePower = Math.copySign(0.1, drivePower);
        }
        mChassis.runSwerve(drivePower,0,0);
    }

    @Override
    public void end(boolean interrupted){
        mChassis.crossWheels();
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
            mChassis.crossWheels();
            finished = true;
        }

        return finished;
    }

}
