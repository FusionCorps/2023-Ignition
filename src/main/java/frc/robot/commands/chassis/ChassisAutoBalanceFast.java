package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisAutoBalanceFast extends CommandBase {

    Chassis mChassis;
    double tilt;
    boolean isStopped;

    public ChassisAutoBalanceFast(Chassis chassis){
        mChassis = chassis;

        addRequirements(mChassis);

        isStopped = false;
    }

    @Override
    public void initialize(){
        isStopped = false;
    }

    @Override
    public void execute(){
        if(mChassis.getPitch()+mChassis.getRoll()>=0) {
            tilt = Math.sqrt(mChassis.getPitch()*mChassis.getPitch()+mChassis.getRoll()*mChassis.getRoll());
        } else{
            tilt = -Math.sqrt(mChassis.getPitch()*mChassis.getPitch()+mChassis.getRoll()*mChassis.getRoll());
        }

        if(tilt>=0) {
            if (isStopped) {
                mChassis.crossWheels();
            } else if (tilt > 8) {
                mChassis.runSwerve(-.15, 0, 0);
                isStopped = false;
            } else {
                isStopped = true;
                mChassis.crossWheels();
            }
        } else{
            if(isStopped){
                mChassis.crossWheels();
            } else if(tilt < -8){
                mChassis.runSwerve(.15,0,0);
                isStopped = false;
            } else{
                isStopped = true;
                mChassis.crossWheels();
            }
        }

//        if(tilt<8 && tilt>-8){
//            mChassis.crossWheels();
//        } else if(tilt>8){
//            mChassis.runSwerve(-0.2,0,0);
//        } else if (tilt < -8){
//            mChassis.runSwerve(0.2,0,0);
//        }
    }
}
