package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterTest extends SubsystemBase {

    WPI_TalonFX shooterTalon;

    public ShooterTest() {
        shooterTalon = new WPI_TalonFX(20);
    }

    public void setShooterTalon(double pct) {
        shooterTalon.set(pct);
    }

}
