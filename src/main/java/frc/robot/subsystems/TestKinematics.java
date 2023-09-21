package frc.robot.subsystems;

import com.team254.lib.geometry.Translation2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.swerve.SwerveDriveKinematics;
import com.team254.lib.swerve.SwerveModuleState;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.TRACK_LENGTH_METERS;
import static frc.robot.Constants.TRACK_WIDTH_METERS;
import static frc.robot.Constants.SWERVE_ROT_SPEED_MAX;
import static frc.robot.Constants.MAX_SPEED;

/*
Tests the 254 kinematics class.
Specifically, 
 */
public class TestKinematics extends SubsystemBase {

    private final Translation2d m_frontLeftLocation = new Translation2d(TRACK_WIDTH_METERS / 2.0,
            TRACK_LENGTH_METERS / 2.0);
    private final Translation2d m_frontRightLocation = new Translation2d(TRACK_WIDTH_METERS / 2.0,
            -TRACK_LENGTH_METERS / 2.0);
    private final Translation2d m_backLeftLocation = new Translation2d(-TRACK_WIDTH_METERS / 2.0,
            TRACK_LENGTH_METERS / 2.0);
    private final Translation2d m_backRightLocation = new Translation2d(-TRACK_WIDTH_METERS / 2.0,
            -TRACK_LENGTH_METERS / 2.0);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    GenericEntry frontLeftEntry;
    GenericEntry frontRightEntry;
    GenericEntry backLeftEntry;
    GenericEntry backRightEntry;

    XboxController controller = new XboxController(0);

    public TestKinematics() {
        frontLeftEntry = Shuffleboard.getTab("Swerve Modules")
                .add("Front left", 0)
                .withWidget(BuiltInWidgets.kGyro)
                .withPosition(0, 0) // specify the widget here
                .getEntry();
        frontRightEntry = Shuffleboard.getTab("Swerve Modules")
                .add("Front right", 0)
                .withWidget(BuiltInWidgets.kGyro)
                .withPosition(2, 0) // specify the widget here
                .getEntry();
        backLeftEntry = Shuffleboard.getTab("Swerve Modules")
                .add("Back left", 0)
                .withWidget(BuiltInWidgets.kGyro)
                .withPosition(0, 2) // specify the widget here
                .getEntry();
        backRightEntry = Shuffleboard.getTab("Swerve Modules")
                .add("Back right", 0)
                .withWidget(BuiltInWidgets.kGyro)
                .withPosition(2, 2) // specify the widget here
                .getEntry();
    }

    @Override
    public void periodic() {

        double translateX = controller.getLeftX();
        double translateY = -controller.getLeftY();

        double rot = controller.getRightX();

        System.out.println(translateX + " " + translateY + " " + rot);

        ChassisSpeeds speeds = new ChassisSpeeds(translateX * MAX_SPEED,
                translateY * MAX_SPEED,
                rot * SWERVE_ROT_SPEED_MAX);

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);

        // the angles of the swerve modules
        double frontLeftAngle = states[0].angle.getDegrees();
        double frontRightAngle = states[1].angle.getDegrees();
        double backLeftAngle = states[2].angle.getDegrees();
        double backRightAngle = states[3].angle.getDegrees();

        // set the angle of the shuffleboard widgets
        frontLeftEntry.setDouble(backRightAngle);
        frontRightEntry.setDouble(frontRightAngle);
        backLeftEntry.setDouble(backLeftAngle);
        backRightEntry.setDouble(frontLeftAngle);
    }
}
