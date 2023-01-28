package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private WPI_TalonFX bicepMotor = new WPI_TalonFX(Constants.BICEP_MOTOR_ID);
    private WPI_TalonFX forearmMotor = new WPI_TalonFX(Constants.FOREARM_MOTOR_ID);

    public Arm() {

        bicepMotor.setNeutralMode(NeutralMode.Brake);
        forearmMotor.setNeutralMode(NeutralMode.Brake);

        bicepMotor.config_kP(0, Constants.BICEP_kP);
        forearmMotor.config_kP(0, Constants.FOREARM_kP);
    }

    // sets bicep to certain angle
    public void moveBicepTo(double angle) {
        bicepMotor.set(TalonFXControlMode.Position, angle);
    }
    // sets the forearm to an angle
    public void moveForearmTo(double angle) {
        forearmMotor.set(TalonFXControlMode.Position, angle);
    }

    // returns the 'angle' of the bicep motor
    public double getBicepAngle() {
        return bicepMotor.getSelectedSensorPosition();
    }
    //returns the 'angle' of forearm motor
    public double getForearmAngle() {
        return forearmMotor.getSelectedSensorPosition();
    }
}

