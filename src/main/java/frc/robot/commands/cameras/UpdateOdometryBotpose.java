package frc.robot.commands.cameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Chassis;

public class UpdateOdometryBotpose extends CommandBase {

    Chassis mChassis;
    Cameras mCameras;

    public UpdateOdometryBotpose(Chassis chassis, Cameras cameras) {
        mChassis = chassis;
        mCameras = cameras;
    }

    @Override
    public void execute() {
        // TODO: Ensure works for both alliances
        // magic numbers added to botpose are to translate coord system

        Pose2d chassisPose = mChassis.getPose();
        double avgX = (chassisPose.getX() + mCameras.botpose()[0] + 8.2296)/2;
        double avgY = (chassisPose.getY() + mCameras.botpose()[1] + 8.2296/2)/2;

        Pose2d averagePose = new Pose2d(avgX, avgY, chassisPose.getRotation());
        mChassis.resetOdometry(averagePose);
    }

}
