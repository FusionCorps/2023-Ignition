package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArmOut extends CommandBase {
    Arm m_arm;

    private final double ERROR_MARGIN = .1;
    private boolean bicepFinished = false;
    private boolean fullyFinished = false;
    double m_bicepAngle;
    double m_forearmAngle;

    public MoveArmOut(Arm arm, double bicepAngle, double forearmAngle) {
        m_arm = arm;
        m_bicepAngle = bicepAngle;
        m_forearmAngle = forearmAngle;
        addRequirements(m_arm);
    }

    private double percentError(double value, double ideal) {
        return Math.abs(value - ideal) / ideal;
    }
    public void initialize() {
        bicepFinished = false;
        fullyFinished = false;
    }
    public void execute() {
        if (!bicepFinished) {
            m_arm.moveBicepTo(m_bicepAngle);
            if (percentError(m_arm.getBicepAngle(), m_bicepAngle) < ERROR_MARGIN) {
                bicepFinished = true;
            }
        } else {
            m_arm.moveForearmTo(m_forearmAngle);
            if(percentError(m_arm.getForearmAngle(), m_forearmAngle) < ERROR_MARGIN) {
                fullyFinished = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return fullyFinished;
    }
}
