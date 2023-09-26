package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

    // configurations for regular controller
    XboxController controller = new XboxController(0);

    // configurations for goofy ahh keyboard 
    XboxController rotational = new XboxController(1);
    boolean isKeyboard = false;


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


    // returns controller input depending on whether you are using the keyboard
    double[] getControllerInput() {
        if (isKeyboard) {
                return new double[]{controller.getLeftX(), controller.getLeftY(), rotational.getRawAxis(0)};
        } 
        return new double[] {controller.getLeftX(), controller.getLeftY(), controller.getRightX()};
    }

    @Override
    public void periodic() {

        double[] input = getControllerInput();

        double translateX = input[0];
        double translateY = input[1];

        double rot = input[2];

        //System.out.println(translateX + " " + translateY + " " + rot);

        ChassisSpeeds speeds = new ChassisSpeeds(translateX * MAX_SPEED,
                translateY * MAX_SPEED,
                rot * SWERVE_ROT_SPEED_MAX);

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);

        // the angles of the swerve modules
        double frontLeftAngle = states[0].angle.getDegrees() + 90;
        double frontRightAngle = states[1].angle.getDegrees() + 90;
        double backLeftAngle = states[2].angle.getDegrees() + 90;
        double backRightAngle = states[3].angle.getDegrees() + 90;

        System.out.println(frontLeftAngle+" "+frontRightAngle+" "+backLeftAngle+" "+backRightAngle);



        // set the angle of the shuffleboard widgets
        frontLeftEntry.setDouble(backRightAngle);
        frontRightEntry.setDouble(frontRightAngle);
        backLeftEntry.setDouble(backLeftAngle);
        backRightEntry.setDouble(frontLeftAngle);
    }
}