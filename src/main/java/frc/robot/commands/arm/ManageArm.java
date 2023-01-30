package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

import static frc.robot.RobotContainer.m_controller;
import static frc.robot.Constants.ArmConstants.*;
import static java.lang.Math.max;
import static java.lang.Math.min;

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
        if (mArm.armAtTarget()) {
            mArm.passSetpoints(mArm.baseTalonTarget, mArm.wristTalonTarget);
        } else if (mArm.wristStowed()) {
            mArm.passSetpoints(mArm.baseTalonTarget, WRIST_STOWED_POS);
        } else {
            mArm.stowWrist();
        }


    }

}
