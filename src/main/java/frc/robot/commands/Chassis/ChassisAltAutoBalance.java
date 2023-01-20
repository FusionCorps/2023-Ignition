package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class ChassisAltAutoBalance extends CommandBase {

    Chassis mChassis;

    Timer endTimer;

    private double error;
    private double drivePower;
    private double currentAngle;

    boolean inMarginofError;

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

        error = Constants.CHARGE_STATION_BALANCE_ANGLE_GOAL - currentAngle;
        drivePower = Math.min(error* Constants.DRIVE_kP,1);

        System.out.println(error + ", " + drivePower + ", " + currentAngle);


        if (Math.abs(drivePower) > 0.2) {
            drivePower = Math.copySign(0.2, drivePower);
          }
      
          mChassis.runSwerve(drivePower,0,0);
    }

    @Override
    public void end(boolean interrupted){
        mChassis.runSwerve(0,0,0);
    }

    @Override
    public boolean isFinished(){
        boolean finished = false;
        inMarginofError = Math.abs(error) < Constants.CHARGE_STATION_BALANCE_ANGLE_GOAL;
        if(inMarginofError){
            endTimer.start();
            if(endTimer.hasElapsed(0.5)){
                finished = true;
                mChassis.crossWheels();
            }else{
                endTimer.reset();
            }
        } else{

            endTimer.reset();
        }

        return finished;
    }

}
