package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

import static java.lang.Math.max;


public class ChassisAutoBalance extends CommandBase {

    // Ok so how this works
    // drive forward (slowly)
    // stop as soon as the platform STARTS tilting and cross
    // avoiding using FC drive for this for now: keeping it simple as possible
    // pitch should DECREASE RAPIDLY once the tilting begins, so once that happens three cycles in a row
    // stop driving and hit the cross.

    Chassis mChassis;

    double priorPitch;

    int triggerCounter = 0;
    boolean isTriggered = false;

    Timer backup_timer;

    public ChassisAutoBalance(Chassis chassis) {
        mChassis = chassis;
        backup_timer = new Timer();

        addRequirements(mChassis);
    }

    @Override
    public void initialize() {
        priorPitch = mChassis.ahrs.getRoll();
        isTriggered = false;
        System.out.println("Command Started");
    }

    @Override
    public void execute() {
        // alternate idea: start tilted up, then trigger on going underneath a certain thresh

        // System.out.println(mChassis.ahrs.getRoll());

        if (mChassis.ahrs.getRoll() < 13) {

            if (!isTriggered) {
                backup_timer.reset();
                backup_timer.start();
            }

            isTriggered = true;
        }




        // don't want to end command, so keep holding wheels locked
        if (isTriggered && backup_timer.hasElapsed(1)) {
            mChassis.crossWheels();
        } else if (isTriggered) {
            mChassis.runSwerve(0.05, 0, 0);
        } else {
            mChassis.runSwerve(-0.15, 0, 0);
        }

    }


}
