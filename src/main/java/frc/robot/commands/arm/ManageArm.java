package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

import static frc.robot.RobotContainer.m_controller;
import static frc.robot.Constants.ArmConstants.*;
import static java.lang.Math.*;

// Main command defining arm logic
// Multi-step algorithm to ensure safety
public class ManageArm extends CommandBase {

    Arm mArm;

    public ManageArm(Arm arm) {
        mArm = arm;

        addRequirements(mArm);
    }

    @Override
    public void execute() {
        // if arm target is on same side as current position, rules change a little.
        // this works but it scares me

        // tested 4-bar like effect but not useful
//        if (mArm.keepParallel) {
//            mArm.wristTalonTarget = -mArm.getBaseTalonPosition() / BASE_GEAR_RATIO * WRIST_GEAR_RATIO;
//        }

        // overriding clause may be deprecated might be ok to remove
        // speedener if arm is same side and moving out ONLY
        if ((mArm.baseTalonTarget * mArm.getBaseTalonPosition() > 0 && (abs(mArm.baseTalonTarget) >= abs(mArm.getBaseTalonPosition())))
                || mArm.overriding) {
            // clause for when arm is out of robot interior and its safe to move both arm and wrist
            // separate check if arm is on intake side going out for faster intake deploy
            if (mArm.armAtTarget() || mArm.safeForDouble()
                    || ((mArm.baseTalonTarget > CHUTE_BASE_POS*4/3) && (mArm.getBaseTalonPosition() > 0) && (mArm.wristTalonTarget < 0))) {
                mArm.passSetpoints(mArm.baseTalonTarget, mArm.wristTalonTarget);
            // if the wrist is inside arm it's safe to move around
            } else if (mArm.wristStowed()) {
                mArm.passSetpoints(mArm.baseTalonTarget, WRIST_STOWED_POS);
            // otherwise tuck the wrist in
            } else {
                mArm.stowWrist();
            }
        } else {
            // no safe for double clause
            // everything else is the same
            if (mArm.armAtTarget()
                    || ((mArm.getBaseTalonPosition() < INTAKE_BASE_POS_CUBE*5/3) && (mArm.baseTalonTarget > CHUTE_BASE_POS*4/3) && (mArm.getBaseTalonPosition() > 0) && (mArm.wristTalonTarget < 0))) {
                mArm.passSetpoints(mArm.baseTalonTarget, mArm.wristTalonTarget);
            } else if (mArm.wristStowed()) {
                mArm.passSetpoints(mArm.baseTalonTarget, WRIST_STOWED_POS);
            } else {
                mArm.stowWrist();
            }
        }


//        if (mArm.keepParallel) {
//            mArm.passSetpoints(mArm.baseTalonTarget, mArm.wristTalonTarget);
//        }


    }

}
