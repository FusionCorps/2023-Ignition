package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    //CANSparkMax intakeMotor;
    WPI_TalonFX intakeMotor;

    private ShuffleboardTab tab = Shuffleboard.getTab("General");

    public GenericEntry voltageFudgeTab = tab.add("Outtake Voltage K", 1.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 2)).getEntry();

    public Intake() {
        //intakeMotor = new CANSparkMax(Constants.INTAKE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        intakeMotor = new WPI_TalonFX(INTAKE_ID);

        // would this help ?
        //intakeMotor.restoreFactoryDefaults();
        // Do we want to invert it?
        // intakeMotor.setInverted(true);

        intakeMotor.setNeutralMode(NeutralMode.Brake);

        intakeMotor.setInverted(TalonFXInvertType.Clockwise);

        voltageFudgeTab.setDouble(1.0);

    }
//
    // sets the motor percent
    public void set(double pct) {
        intakeMotor.set(pct);
    }

    // added voltage comp for consistency
    public void setVolts(double volts) {
        intakeMotor.setVoltage(volts*voltageFudgeTab.getDouble(1.0));
    }
}
