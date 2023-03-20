package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

import static frc.robot.RobotContainer.m_controller;
import static frc.robot.Constants.ArmConstants.*;
import static java.lang.Math.*;

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
//        if (mArm.baseTalonTarget*mArm.getBaseTalonPosition() >= 0) {
//            if (mArm.armAtTarget() || mArm.safeForDouble()) {
//                mArm.passSetpoints(mArm.baseTalonTarget, mArm.wristTalonTarget);
//            } else if (mArm.wristStowed()) {
//                mArm.passSetpoints(mArm.baseTalonTarget, WRIST_STOWED_POS);
//            } else {
//                mArm.stowWrist();
//            }
//        } else {
//            if (mArm.armAtTarget()) {
//                mArm.passSetpoints(mArm.baseTalonTarget, mArm.wristTalonTarget);
//            } else if (mArm.wristStowed()) {
//                mArm.passSetpoints(mArm.baseTalonTarget, WRIST_STOWED_POS);
//            } else {
//                mArm.stowWrist();

//            }
//        }

        if (mArm.keepParallel) {
            mArm.wristTalonTarget = -mArm.getBaseTalonPosition() / BASE_GEAR_RATIO * WRIST_GEAR_RATIO;
        }

        // speedener if arm is same side and moving out ONLY
        if ((mArm.baseTalonTarget * mArm.getBaseTalonPosition() > 0 && (abs(mArm.baseTalonTarget) >= abs(mArm.getBaseTalonPosition())))
                || mArm.overriding) {
            if (mArm.armAtTarget() || mArm.safeForDouble()) {
                mArm.passSetpoints(mArm.baseTalonTarget, mArm.wristTalonTarget);
                if (mArm.safeForDouble()) {
                    // System.out.println("doubling");
                }
            } else if (mArm.wristStowed()) {
                mArm.passSetpoints(mArm.baseTalonTarget, WRIST_STOWED_POS);
            } else {
                mArm.stowWrist();
            }
        } else {
            if (mArm.armAtTarget()) {
                mArm.passSetpoints(mArm.baseTalonTarget, mArm.wristTalonTarget);
            } else if (mArm.wristStowed()) {
                mArm.passSetpoints(mArm.baseTalonTarget, WRIST_STOWED_POS);
            } else {
                mArm.stowWrist();
            }
        }

        // pseudocode for fast intake movement check
        // ((base on intake side) AND (wrist in same direction as base)) OR (everything else)
        // ((mArm.getBaseTalonPosition() > 0) && (mArm.wristTalonTarget < 0)) || (everything else)
        // TODO: Condense logic into single function

        // speedener was too fast
//        if (mArm.armAtTarget()) {
//            mArm.passSetpoints(mArm.baseTalonTarget, mArm.wristTalonTarget);
//            System.out.println("Arm at target");
//        } else if (mArm.wristStowed()) {
//            mArm.passSetpoints(mArm.baseTalonTarget, WRIST_STOWED_POS);
//        } else {
//            mArm.stowWrist();
//        }

        if (mArm.keepParallel) {
            mArm.passSetpoints(mArm.baseTalonTarget, mArm.wristTalonTarget);
        }


    }

}
