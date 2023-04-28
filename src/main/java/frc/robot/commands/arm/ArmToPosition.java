package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

import static frc.robot.Constants.ArmConstants.*;

// Command for moving arm to setpoints in auto
// since CommandGroups would override DefaultCommand
public class ArmToPosition extends CommandBase {

    Arm mArm;

    double baseTarget;
    double wristTarget;

    Timer timer = new Timer();
    boolean timerStarted = false;

    double delay;

    // overloaded constructor so default delay is 0.25
    // there's a way better way to do this I'm just lazy
    public ArmToPosition(Arm arm, double basePos, double wristPos) {
        mArm = arm;

        baseTarget = basePos;
        wristTarget = wristPos;

        delay = 0.25;
    }

    public ArmToPosition(Arm arm, double basePos, double wristPos, double delayTime) {
        mArm = arm;

        baseTarget = basePos;
        wristTarget = wristPos;

        delay = delayTime;
    }

    // reset timer and stop so the command doesn't end prematurely
    @Override
    public void initialize() {
        mArm.setTalonTargets(baseTarget, wristTarget);
        timer.reset();
        timer.stop();
        timerStarted = false;
    }

    @Override
    public void execute() {

//        mArm.baseTalonTarget = HIGH_BASE_POS_ALT_PREP;
//        mArm.wristTalonTarget = HIGH_WRIST_POS_ALT;

        // arm logic copy pasted from ManageArm
        // minus some of the more advanced cases
        if (mArm.armAtTarget()) {
            mArm.passSetpoints(mArm.baseTalonTarget, mArm.wristTalonTarget);
        } else if (mArm.wristStowed()) {
            mArm.passSetpoints(mArm.baseTalonTarget, WRIST_STOWED_POS);
        } else {
            mArm.stowWrist();
        }

        // start timer once arm reaches target
        if (!timerStarted && (mArm.armAtTarget() && mArm.wristAtTarget())) {
            timerStarted = true;
            timer.reset();
            timer.start();
        }
    }

    // end command once delay is over
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(delay);
    }

    // print out for testing (old)
    @Override
    public void end(boolean isFinished) {
        // System.out.println("ArmToPosition Finished");
    }

}
