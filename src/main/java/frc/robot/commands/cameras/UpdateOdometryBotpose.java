package frc.robot.commands.cameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Chassis;

public class UpdateOdometryBotpose extends CommandBase {

    Chassis mChassis;
    Cameras mCameras;

    String tableKey;
    double[] visMeas;

    public UpdateOdometryBotpose(Chassis chassis, Cameras cameras, boolean isRed) {
        mChassis = chassis;
        mCameras = cameras;

        if (isRed) {
            tableKey = "botpose_wpired";
        } else {
            tableKey = "botpose_wpiblue";
        }
    }

    @Override
    public void execute() {
        // TODO: Ensure works for both alliances
        // magic numbers added to botpose are to translate coord system

//        Pose2d chassisPose = mChassis.getPose();
//        double avgX = (chassisPose.getX() + mCameras.botpose()[0] + 8.2296)/2;
//        double avgY = (chassisPose.getY() + mCameras.botpose()[1] + 8.2296/2)/2;
//
//        Pose2d averagePose = new Pose2d(avgX, avgY, chassisPose.getRotation());
//        mChassis.resetOdometry(averagePose);

        visMeas = mCameras.ll_table.getEntry(tableKey).getDoubleArray(new double[6]);

        Pose2d measuredPose = new Pose2d(visMeas[0], visMeas[1], mChassis.getPose().getRotation());

        mChassis.passVisionMeasurement(measuredPose);

    }

}
