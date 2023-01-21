package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;
import static java.lang.Math.PI;
import static java.lang.Math.cos;

public class Arm extends SubsystemBase {

    WPI_TalonFX baseTalon;
    WPI_TalonFX wristTalon;

    public Arm() {
        baseTalon = new WPI_TalonFX(BASE_ID);
        wristTalon = new WPI_TalonFX(WRIST_ID);

        baseTalon.setSelectedSensorPosition(BASE_START_POS);

        baseTalon.config_kF(0, BASE_kF);
        baseTalon.config_kP(0, BASE_kP);
        baseTalon.config_kI(0, BASE_kI);
        baseTalon.config_kD(0, BASE_kD);

        wristTalon.setSelectedSensorPosition(WRIST_START_POS);

        wristTalon.config_kF(0, WRIST_kF);
        wristTalon.config_kP(0, WRIST_kP);
        wristTalon.config_kI(0, WRIST_kI);
        wristTalon.config_kD(0, WRIST_kD);
    }

    public void passSetpoints(int basePos, int wristPos) {
        baseTalon.set(TalonFXControlMode.Position, basePos);
        wristTalon.set(TalonFXControlMode.Position, wristPos);
    }

    public void passSetpointsFF(int basePos, int wristPos) {
        double baseAngleRad = baseTalon.getSelectedSensorPosition()*PI/1024/BASE_GEAR_RATIO;
        double wristAngleRad = wristTalon.getSelectedSensorPosition()*PI/1024/WRIST_GEAR_RATIO;

        double baseFFAdj = BASE_FF*cos(baseAngleRad) + WRIST_FF*cos(baseAngleRad+wristAngleRad);
        double wristFFAdj = WRIST_FF*cos(baseAngleRad+wristAngleRad);

        baseTalon.set(TalonFXControlMode.Position, basePos, DemandType.ArbitraryFeedForward, baseFFAdj);
        wristTalon.set(TalonFXControlMode.Position, wristPos, DemandType.ArbitraryFeedForward, wristFFAdj);
    }

}
