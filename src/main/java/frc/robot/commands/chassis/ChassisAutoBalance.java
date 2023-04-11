package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

import static java.lang.Math.max;


public class ChassisAutoBalance extends CommandBase {

    // drive forward until under thresh
    // backup and then X modules

    Chassis mChassis;

    boolean isTriggered = false;

    Timer backupTimer;

    public ChassisAutoBalance(Chassis chassis) {
        mChassis = chassis;
        backupTimer = new Timer();

        addRequirements(mChassis);
    }

    @Override
    public void initialize() {
        isTriggered = false;
        System.out.println("Command Started");
    }

    @Override
    public void execute() {
        // alternate idea: start tilted up, then trigger on going underneath a certain thresh
        // this worked better, continue researching

        System.out.println(mChassis.ahrs.getPitch());

        if (mChassis.ahrs.getPitch() < 12.75) {

            if (!isTriggered) {
                backupTimer.reset();
                backupTimer.start();
            }

            isTriggered = true;
        }

        // don't want to end command, so keep holding wheels locked
        if (isTriggered && backupTimer.hasElapsed(0.75)) {
            mChassis.crossWheels();
        } else if (isTriggered) {
            mChassis.runSwerve(0.13, 0, 0);
        } else {
            mChassis.runSwerve(-0.22, 0, 0);
        }

    }


}
