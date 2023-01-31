package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

import static frc.robot.RobotContainer.m_controller;
import static frc.robot.Constants.ArmConstants.*;
import static java.lang.Math.*;

public class ManageArm extends CommandBase {

    Arm mArm;

    // note to self - VERY DANGEROUS
    double baseFudge;

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

        // speedener if arm is same side and moving out ONLY
        if (mArm.baseTalonTarget*mArm.getBaseTalonPosition() >= 0 && abs(mArm.baseTalonTarget) >= abs(mArm.getBaseTalonPosition())) {
            if (mArm.armAtTarget() || mArm.safeForDouble()) {
                mArm.passSetpoints(mArm.baseTalonTarget, mArm.wristTalonTarget);
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

    }

}
