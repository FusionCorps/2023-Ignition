package frc.robot.commands.Chassis;

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

    public ChassisAutoBalance(Chassis chassis) {
        mChassis = chassis;

        addRequirements(mChassis);
    }

    @Override
    public void initialize() {
        priorPitch = mChassis.ahrs.getPitch();
    }

    @Override
    public void execute() {
        // alternate idea: start tilted up, then trigger on going underneath a certain thresh
        if (mChassis.ahrs.getPitch() < priorPitch) {
            triggerCounter++;
        } else {
            triggerCounter--;
        }

        triggerCounter = max(triggerCounter, 0);

        if (triggerCounter > 3) {
            isTriggered = true;
        }

        // don't want to end command, so keep holding wheels locked
        if (isTriggered) {
            mChassis.crossWheels();
        } else {
            mChassis.runSwerve(0.2, 0, 0);
        }
    }


}
