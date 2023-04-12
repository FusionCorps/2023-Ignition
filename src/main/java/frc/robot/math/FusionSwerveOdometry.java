package frc.robot.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class FusionSwerveOdometry {

    Pose2d robotPose; // store values in a Pose2D

    SwerveModulePosition[] lastPositions; // save last known positions

    public FusionSwerveOdometry(double xInit, double yInit, Rotation2d rotInit, SwerveModulePosition[] initSwervePositions) {
        robotPose = new Pose2d(xInit, yInit, rotInit);

        lastPositions = initSwervePositions;
    }

    // update with new swerve positions and gyro data
    public void updateCustomOdo(Rotation2d newRot, SwerveModulePosition[] newPos) {

        double[] deltaFL = getPosDelta(newPos[0], lastPositions[0]);
        double[] deltaBL = getPosDelta(newPos[1], lastPositions[1]);
        double[] deltaFR = getPosDelta(newPos[2], lastPositions[2]);
        double[] deltaBR = getPosDelta(newPos[3], lastPositions[3]);

        // rotation vectors should average out
        double netDeltaX = (deltaFL[0] + deltaBL[0] + deltaFR[0] + deltaBR[0])/4;
        double netDeltaY = (deltaFL[1] + deltaBL[1] + deltaFR[1] + deltaBR[1])/4;

        robotPose = new Pose2d(
                robotPose.getX() + netDeltaX*newRot.getCos() - netDeltaY*newRot.getSin(),
                robotPose.getY() + netDeltaY*newRot.getCos() + netDeltaX* newRot.getSin(),
                newRot
        );

        lastPositions = newPos;
    }

    // convenience function to get the difference in swerve modules
    public double[] getPosDelta(SwerveModulePosition newPos, SwerveModulePosition oldPos) {
        double xDelta = (newPos.distanceMeters - oldPos.distanceMeters)*cos(newPos.angle.getRadians());
        double yDelta = (newPos.distanceMeters - oldPos.distanceMeters)*sin(newPos.angle.getRadians());

        return new double[] {xDelta, yDelta};
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    public void resetPosition() {

    }

}
