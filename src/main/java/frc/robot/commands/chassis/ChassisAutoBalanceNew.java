package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisAutoBalanceNew extends CommandBase {
    // drive forward until under thresh
    // backup and then X modules

    Chassis mChassis;

    double speedK = 1.0;

    boolean isTriggered = false;
    boolean isFinished = false;

    Timer backupTimer;
    Timer checkTimer;

    public ChassisAutoBalanceNew(Chassis chassis) {
        mChassis = chassis;
        backupTimer = new Timer();
        checkTimer = new Timer();

        addRequirements(mChassis);
    }

    @Override
    public void initialize() {
        isTriggered = false;
        System.out.println("Command Started");
        speedK = 1.0;
    }

    @Override
    public void execute() {
        // alternate idea: start tilted up, then trigger on going underneath a certain thresh
        // this worked better, continue researching

        System.out.println(backupTimer.get());

        if ((mChassis.ahrs.getPitch() < 5.75 && mChassis.ahrs.getPitch() > -5.75)) {
            mChassis.crossWheels();
            isTriggered = true;
        } else if (mChassis.ahrs.getPitch() > 5.75) {
            mChassis.runSwerve(-0.2*speedK, 0, 0);
            if (isTriggered) {
                isTriggered = false;
                speedK /= 1.6;
            }
        } else if (mChassis.ahrs.getPitch() < 5.75) {
            mChassis.runSwerve(0.2*speedK, 0, 0);
            if (isTriggered) {
                isTriggered = false;
                speedK /= 1.6;
            }
        }

        // don't want to end command, so keep holding wheels locked
//        if (isTriggered && backupTimer.hasElapsed(0.75)) {
//            mChassis.crossWheels();
//        } else if (isTriggered) {
//            mChassis.runSwerve(0.13, 0, 0);
//        } else {
//            mChassis.runSwerve(-0.22, 0, 0);
//        }

    }



}
