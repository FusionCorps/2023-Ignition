package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.IntakeConstants.OUTTAKE_VOLTS;

public class CubeFling extends CommandBase {

    // assumes start at 0 -70
    // move arm to 0 +70
    // release cube at 0 0

    // used in tele only so no need to recreate arm motion logic here

    // logic pseudocode
    //
    // bool init? = false
    //
    //
    // if (!init) {
    //     if (arm != 0 -70):
    //          arm -> 0 -70
    //     else:
    //          init = true
    // }

    // else:
    //      arm -> 0 70
    //      if arm > 0 0:
    //          mIntake.set(out)

    // end when arm at 0 70 and init = true

    Intake mIntake;
    Arm mArm;

    boolean hasInit = false;

    public CubeFling(Intake intake, Arm arm) {
        mIntake = intake;
        mArm = arm;

        // allow for arm to controlled by ManageArm
        addRequirements(mIntake);
    }

    @Override
    public void initialize() {
        // reset init state
        hasInit = false;
        mArm.setTalonTargets(FLING_BASE_POS, FLING_INIT_WRIST_POS);

        // set intake to hold cubes
        mIntake.set(-0.2);
    }

    @Override
    public void execute() {
        if (!hasInit) {
            if (mArm.isAt(FLING_BASE_POS, FLING_INIT_WRIST_POS)) {
                hasInit = true;
            } else {
                mArm.setTalonTargets(FLING_BASE_POS, FLING_INIT_WRIST_POS);
            }
        } else {
            mArm.setTalonTargets(FLING_BASE_POS, FLING_FINAL_WRIST_POS);
            if (mArm.getWristPos() > 0) {
                mIntake.setVolts(OUTTAKE_VOLTS);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return (mArm.isAt(FLING_BASE_POS, FLING_FINAL_WRIST_POS) && hasInit == true);
    }

    @Override
    public void end(boolean isFinished) {
//        mArm.setTalonTargets(0, 0);
        mArm.setTalonTargets(mArm.getBasePos(), mArm.getWristPos());
    }

}
