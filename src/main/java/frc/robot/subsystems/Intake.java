package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    //CANSparkMax intakeMotor;
    WPI_TalonFX intakeMotor;

    public Intake() {
        //intakeMotor = new CANSparkMax(Constants.INTAKE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        intakeMotor = new WPI_TalonFX(INTAKE_ID);

        // would this help ?
        //intakeMotor.restoreFactoryDefaults();
        // Do we want to invert it?
        // intakeMotor.setInverted(true);
    }
//
    // sets the motor percent
    public void set(double pct) {
        intakeMotor.set(pct);
    }
}
