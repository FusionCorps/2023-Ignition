package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// anything involving vision will go here
public class Cameras extends SubsystemBase {

    public NetworkTable ll_table;

    public Cameras() {
        ll_table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double tx() {
        return ll_table.getEntry("tx").getDouble(0.0);
    }

    public double ty() {
        return ll_table.getEntry("ty").getDouble(0.0);
    }

}
