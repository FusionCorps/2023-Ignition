package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;
import static java.lang.Math.*;

public class Arm extends SubsystemBase {

    public double baseTalonTarget = 1000;
    public double wristTalonTarget = 0;

    WPI_TalonFX baseTalon;
    WPI_TalonFX wristTalon;

    WPI_TalonFX baseFollower;

    public Arm() {
        baseTalon = new WPI_TalonFX(BASE_ID);
        wristTalon = new WPI_TalonFX(WRIST_ID);

        baseFollower = new WPI_TalonFX(BASE_FOLLOWER_ID);

        baseTalon.setSelectedSensorPosition(BASE_START_POS);

        baseTalon.setNeutralMode(NeutralMode.Brake);

        baseTalon.config_kF(0, BASE_kF);
        baseTalon.config_kP(0, BASE_kP);
        baseTalon.config_kI(0, BASE_kI);
        baseTalon.config_kD(0, BASE_kD);

        baseTalon.configMotionCruiseVelocity(BASE_MAX_V);
        baseTalon.configMotionAcceleration(BASE_MAX_A);
        baseTalon.configMotionSCurveStrength(BASE_CURVE_STR);

        baseFollower.setNeutralMode(NeutralMode.Brake);
        baseFollower.follow(baseTalon);
        baseFollower.setInverted(InvertType.FollowMaster);

        wristTalon.setSelectedSensorPosition(WRIST_START_POS);

        wristTalon.setNeutralMode(NeutralMode.Brake);

        wristTalon.config_kF(0, WRIST_kF);
        wristTalon.config_kP(0, WRIST_kP);
        wristTalon.config_kI(0, WRIST_kI);
        wristTalon.config_kD(0, WRIST_kD);

        wristTalon.configMotionCruiseVelocity(WRIST_MAX_V);
        wristTalon.configMotionAcceleration(WRIST_MAX_A);
        wristTalon.configMotionSCurveStrength(WRIST_CURVE_STR);
    }

//    @Override
//    public void periodic() {
//        System.out.println(wristTalon.getSelectedSensorPosition()*180/1024/WRIST_GEAR_RATIO);
//    }

    public void passSetpoints(double basePos, double wristPos) {
        baseTalon.set(TalonFXControlMode.MotionMagic, basePos);
        wristTalon.set(TalonFXControlMode.MotionMagic, wristPos);
    }

    public double getBaseTalonPosition() {
        return baseTalon.getSelectedSensorPosition();
    }

    public void passSetpointsFF(int basePos, int wristPos) {
        double baseAngleRad = baseTalon.getSelectedSensorPosition()*PI/1024/BASE_GEAR_RATIO;
        double wristAngleRad = wristTalon.getSelectedSensorPosition()*PI/1024/WRIST_GEAR_RATIO;

        double baseFFAdj = BASE_FF*cos(baseAngleRad) + WRIST_FF*cos(baseAngleRad+wristAngleRad);
        double wristFFAdj = WRIST_FF*cos(baseAngleRad+wristAngleRad);

        baseTalon.set(TalonFXControlMode.Position, basePos, DemandType.ArbitraryFeedForward, baseFFAdj);
        wristTalon.set(TalonFXControlMode.Position, wristPos, DemandType.ArbitraryFeedForward, wristFFAdj);
    }

    public void setTalonTargets(double baseTarget, double wristTarget) {
        baseTalonTarget = baseTarget;
        wristTalonTarget = wristTarget;
    }

    public boolean armAtTarget() {
        return (abs(baseTalon.getSelectedSensorPosition() - baseTalonTarget) < BASE_ERROR_THRESHOLD);
    }

    public boolean safeForDouble() {
        return (abs(baseTalon.getSelectedSensorPosition()) > BASE_SAFETY_THRESHOLD);
    }

    public boolean wristStowed() {
        return (abs(wristTalon.getSelectedSensorPosition() - WRIST_STOWED_POS) < WRIST_ERROR_THRESHOLD);
    }

    public void stowWrist() {
        baseTalon.set(TalonFXControlMode.PercentOutput, 0);
        wristTalon.set(TalonFXControlMode.MotionMagic, WRIST_STOWED_POS);
        System.out.println("Stowing Wrist");
    }

    public void setMotorsBrake() {
        baseTalon.set(0);
        wristTalon.set(0);
    }

}
