package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IS_LOGGING;

// anything involving vision will go here
public class Cameras extends SubsystemBase {

    public NetworkTable ll_table;

    boolean aimingAtRRTape = true;

    DoubleLogEntry xPosLog;
    DoubleLogEntry yPosLog;
    DoubleLogEntry rotPosLog;

    public Cameras() {
        ll_table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
        // double[] botpose = ll_table.getEntry("botpose").getDoubleArray(new double[] {0.0, 0.0});

//        String ret_str = "";
//
//        for (double value: botpose) {
//            ret_str += value;
//            ret_str += "|";
//        }
//
//        System.out.println(ret_str);

//        if (IS_LOGGING) {
//            xPosLog.append(botpose()[0]);
//            yPosLog.append(botpose()[1]);
//        }
    }

    public double tx() {
        return ll_table.getEntry("tx").getDouble(0.0);
    }

    public double ty() {
        return ll_table.getEntry("ty").getDouble(0.0);
    }

    public double[] botpose() {
        return ll_table.getEntry("botpose").getDoubleArray(new double[] {0.0, 0.0});
    }

    // assuming we are using 0/1 for tape/tag pipelines, respectively
    // for more pipelines, you could consider keeping these in Constants
    public void togglePipeline() {
        if (aimingAtRRTape) {
            ll_table.getEntry("pipeline").setNumber(1);
        } else {
            ll_table.getEntry("pipeline").setNumber(0);
        }

        aimingAtRRTape = !aimingAtRRTape;
    }

}
