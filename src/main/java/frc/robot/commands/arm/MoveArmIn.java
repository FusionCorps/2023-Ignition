package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

// Moves arm to safe zone
public class MoveArmIn extends CommandBase {
    private Arm m_arm;

    private final double ERROR_MARGIN = .1;
    private boolean forearmFinished = false;
    private boolean fullyFinished   = false;
    public MoveArmIn(Arm arm){
        m_arm = arm;
        addRequirements(m_arm);
    }

    // returns generic percent error
    private double percentError(double value, double ideal) {
        return Math.abs(value - ideal) / ideal;
    }

    // resets the cutoff booleans
    @Override
    public void initialize() {
        fullyFinished = false;
        forearmFinished = false;
    }
    @Override
    public void execute() {
        if (!forearmFinished) {
            m_arm.moveForearmTo(Constants.SAFE_FOREARM_ANGLE);
            if (percentError(m_arm.getForearmAngle(), Constants.SAFE_FOREARM_ANGLE) < ERROR_MARGIN) {
                forearmFinished = true;
            }
        } else {
            m_arm.moveBicepTo(Constants.SAFE_BICEP_ANGLE);
            if (percentError(m_arm.getBicepAngle(), Constants.SAFE_BICEP_ANGLE) < ERROR_MARGIN) {
                fullyFinished = true;
            }
        }
    }
    public boolean isFinished() {
        return fullyFinished;
    }
}
