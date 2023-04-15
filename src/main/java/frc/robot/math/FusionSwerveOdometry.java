package frc.robot.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import static frc.robot.Constants.TRACK_LENGTH_METERS;
import static frc.robot.Constants.TRACK_WIDTH_METERS;
import static java.lang.Math.*;

public class FusionSwerveOdometry {

    Pose2d robotPose; // store values in a Pose2D

    SwerveModulePosition[] lastPositions; // save last known positions
    Rotation2d lastRotation;

    Translation2d[] moduleLocations;

    public FusionSwerveOdometry(double xInit, double yInit, Rotation2d rotInit, SwerveModulePosition[] initSwervePositions) {
        robotPose = new Pose2d(xInit, yInit, rotInit);

        lastPositions = initSwervePositions;
        lastRotation = rotInit;

        // starting locations of the modules
        // FL BL FR BR
        moduleLocations = new Translation2d[] {
                new Translation2d(xInit + TRACK_WIDTH_METERS/2*rotInit.getCos() - TRACK_LENGTH_METERS/2* rotInit.getSin(),
                        yInit + TRACK_LENGTH_METERS/2*rotInit.getCos() + TRACK_WIDTH_METERS/2* rotInit.getSin()),
                new Translation2d(xInit - TRACK_WIDTH_METERS/2*rotInit.getCos() - TRACK_LENGTH_METERS/2* rotInit.getSin(),
                        yInit + TRACK_LENGTH_METERS/2*rotInit.getCos() - TRACK_WIDTH_METERS/2* rotInit.getSin()),
                new Translation2d(xInit + TRACK_WIDTH_METERS/2*rotInit.getCos() + TRACK_LENGTH_METERS/2* rotInit.getSin(),
                        yInit - TRACK_LENGTH_METERS/2*rotInit.getCos() + TRACK_WIDTH_METERS/2* rotInit.getSin()),
                new Translation2d(xInit - TRACK_WIDTH_METERS/2*rotInit.getCos() + TRACK_LENGTH_METERS/2* rotInit.getSin(),
                        yInit - TRACK_LENGTH_METERS/2*rotInit.getCos() - TRACK_WIDTH_METERS/2* rotInit.getSin())
        };
    }

    // update with new swerve positions and gyro data
    public void updateCustomOdo(Rotation2d newRot, SwerveModulePosition[] newPos) {
        // get changes between last and current positions
        double[] deltaFL = getPosDelta(newPos[0], lastPositions[0]);
        double[] deltaBL = getPosDelta(newPos[1], lastPositions[1]);
        double[] deltaFR = getPosDelta(newPos[2], lastPositions[2]);
        double[] deltaBR = getPosDelta(newPos[3], lastPositions[3]);

        // rotation vectors should average out
        double netDeltaX = (deltaFL[0] + deltaBL[0] + deltaFR[0] + deltaBR[0])/4;
        double netDeltaY = (deltaFL[1] + deltaBL[1] + deltaFR[1] + deltaBR[1])/4;

        double netDistance = sqrt(pow(netDeltaX,2) + pow(netDeltaY,2));
        double netDeltaTheta = abs(newRot.getRadians() - lastRotation.getRadians());

        // use heading to correct robot relative translation to field relative
        // add fudge factor ~ 1 cm per meter
        //
        // -0.0125 dX
        robotPose = new Pose2d(
                robotPose.getX() + netDeltaX*newRot.getCos() - netDeltaY*newRot.getSin()
                        - 0.0065*abs(netDeltaX) - 0.0065*abs(netDeltaY),
                robotPose.getY() + netDeltaY*newRot.getCos() + netDeltaX* newRot.getSin()
                        - 0.0125*abs(netDeltaX) - 0.0065*abs(netDeltaY)
                , newRot
        );

        // fudge factor

        updateModulePositions(newPos, newRot);

        // save readings as last read positions
        lastPositions = newPos;
        lastRotation = newRot;
    }

    // reset to a passed Pose2d
    public void resetOdometryPoseOnly(Pose2d newPose) {
        Rotation2d rotInit = newPose.getRotation();
        robotPose = newPose;

        double xInit = newPose.getX();
        double yInit = newPose.getY();

        // starting locations of the modules
        // FL BL FR BR
        moduleLocations = new Translation2d[] {
                new Translation2d(xInit + TRACK_WIDTH_METERS/2*rotInit.getCos() - TRACK_LENGTH_METERS/2* rotInit.getSin(),
                        yInit + TRACK_LENGTH_METERS/2*rotInit.getCos() + TRACK_WIDTH_METERS/2* rotInit.getSin()),
                new Translation2d(xInit - TRACK_WIDTH_METERS/2*rotInit.getCos() - TRACK_LENGTH_METERS/2* rotInit.getSin(),
                        yInit + TRACK_LENGTH_METERS/2*rotInit.getCos() - TRACK_WIDTH_METERS/2* rotInit.getSin()),
                new Translation2d(xInit + TRACK_WIDTH_METERS/2*rotInit.getCos() + TRACK_LENGTH_METERS/2* rotInit.getSin(),
                        yInit - TRACK_LENGTH_METERS/2*rotInit.getCos() + TRACK_WIDTH_METERS/2* rotInit.getSin()),
                new Translation2d(xInit - TRACK_WIDTH_METERS/2*rotInit.getCos() + TRACK_LENGTH_METERS/2* rotInit.getSin(),
                        yInit - TRACK_LENGTH_METERS/2*rotInit.getCos() - TRACK_WIDTH_METERS/2* rotInit.getSin())
        };
    }

    // convenience function to get the difference in swerve modules
    public double[] getPosDelta(SwerveModulePosition newPos, SwerveModulePosition oldPos) {
        double xDelta = (newPos.distanceMeters - oldPos.distanceMeters)*cos((newPos.angle.getRadians() + newPos.angle.getRadians())/2);
        double yDelta = (newPos.distanceMeters - oldPos.distanceMeters)*sin((newPos.angle.getRadians() + newPos.angle.getRadians())/2);

        return new double[] {xDelta, yDelta};
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    public void resetPosition(Rotation2d rot, SwerveModulePosition[] positions, Pose2d pose) {
        robotPose = pose;
        lastPositions = positions;
    }

    // updating array of module positions
    // might consider changing to Pose2d instead
    private void updateModulePositions(SwerveModulePosition[] newPos, Rotation2d heading) {
        // iterate through array
        for (int i = 0; i < newPos.length; i++) {
            // get distance traveled by wheel since last update
            double deltaD = newPos[i].distanceMeters - lastPositions[i].distanceMeters;

            // using angle of wheel, find robot-relative translation
            double deltaX = deltaD*cos((newPos[i].angle.getRadians() + lastPositions[i].angle.getRadians())/2);
            double deltaY = deltaD*sin((newPos[i].angle.getRadians() + lastPositions[i].angle.getRadians())/2);

            // use heading to convert to field-relative translation
            moduleLocations[i] = new Translation2d(moduleLocations[i].getX() + deltaX*heading.getCos() - deltaY*heading.getSin(),
                    moduleLocations[i].getY() + deltaY*heading.getCos() + deltaX*heading.getSin());
        }
    }

    // method to visualize module positions
    public void pushModuleLocationsToField(Field2d field) {
        field.getObject("FL Module").setPose(moduleLocations[0].getX(), moduleLocations[0].getY(), new Rotation2d());
        field.getObject("BL Module").setPose(moduleLocations[1].getX(), moduleLocations[1].getY(), new Rotation2d());
        field.getObject("FR Module").setPose(moduleLocations[2].getX(), moduleLocations[2].getY(), new Rotation2d());
        field.getObject("BR Module").setPose(moduleLocations[3].getX(), moduleLocations[3].getY(), new Rotation2d());
    }


    // convenience function to get the translational difference traveled by the wheel
    // assumes the wheel travels in an arc given an unknown period
    public double getAdjDistance(SwerveModulePosition newPosition, SwerveModulePosition oldPosition) {
        // difference in wheel encoder position for drive and steer
        double wheelTravel = newPosition.distanceMeters - oldPosition.distanceMeters;
        double angleDifference = newPosition.angle.getRadians() - oldPosition.angle.getRadians();

        // avoid divide by zero error
        if (angleDifference == 0) {
            return wheelTravel;
        } else {
            // quick maths:
            // radius*theta = distance
            // translation = 2*radius*sin(theta/2)
            // in this case, theta = difference in angles
            // so translation = 2*distance*sin(theta/2)/theta
            return wheelTravel*2*sin(angleDifference/2)/angleDifference;
        }
    }

}
