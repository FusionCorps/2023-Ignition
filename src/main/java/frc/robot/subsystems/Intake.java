package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    CANSparkMax intakeMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(Constants.INTAKE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        
        // would this help ?
        intakeMotor.restoreFactoryDefaults();
        // Do we want to invert it?
        // intakeMotor.setInverted(true);
    }

    // sets the motor percent
    public void set(double pct) {
        intakeMotor.set(pct);
    }
}
