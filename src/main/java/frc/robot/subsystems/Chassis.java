package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
<<<<<<< Updated upstream
=======
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.math.SwerveCalcs;
import frc.robot.modules.SwerveCombo;

import static frc.robot.Constants.*;
import static frc.robot.math.SwerveCalcs.getAngle;
import static frc.robot.math.SwerveCalcs.getSpeed;
import static java.lang.Double.max;
import static java.lang.Math.PI;

public class Chassis extends SubsystemBase {

    // swerve motor + coder init
    private static WPI_TalonFX drive0 = new WPI_TalonFX(DRIVE_FL_ID);
    private static WPI_TalonFX drive1 = new WPI_TalonFX(DRIVE_BL_ID);
    private static WPI_TalonFX drive2 = new WPI_TalonFX(DRIVE_FR_ID);
    private static WPI_TalonFX drive3 = new WPI_TalonFX(DRIVE_BR_ID);

    private static WPI_TalonFX axis0 = new WPI_TalonFX(AXIS_FL_ID);
    private static WPI_TalonFX axis1 = new WPI_TalonFX(AXIS_BL_ID);
    private static WPI_TalonFX axis2 = new WPI_TalonFX(AXIS_FR_ID);
    private static WPI_TalonFX axis3 = new WPI_TalonFX(AXIS_BR_ID);

    CANCoder coder0 = new CANCoder(Constants.CODER_FL_ID);
    CANCoder coder1 = new CANCoder(Constants.CODER_BL_ID);
    CANCoder coder2 = new CANCoder(Constants.CODER_FR_ID);
    CANCoder coder3 = new CANCoder(Constants.CODER_BR_ID);

    public SwerveCombo comboFL = new SwerveCombo(axis0, drive0, coder0, 0);
    public SwerveCombo comboBL = new SwerveCombo(axis1, drive1, coder1, 1);
    public SwerveCombo comboFR = new SwerveCombo(axis2, drive2, coder2, 2);
    public SwerveCombo comboBR = new SwerveCombo(axis3, drive3, coder3, 3);

    public static AHRS ahrs = new AHRS(SPI.Port.kMXP);

    boolean isPrecision = false;

<<<<<<< Updated upstream
=======
    DoubleLogEntry xPosLog;
    DoubleLogEntry yPosLog;
    DoubleLogEntry rotPosLog;

    DoubleLogEntry aprilXPosLog;
    DoubleLogEntry aprilYPosLog;
    DoubleLogEntry aprilRotPosLog;




    NetworkTableEntry limelightField = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_blue");
    DoubleLogEntry deltaXPosLog;
    DoubleLogEntry deltaYPosLog;
    DoubleLogEntry deltaRotPosLog;

>>>>>>> Stashed changes
    // odometry set-up (import - Ri3D Redux)
    private final Translation2d m_frontLeftLocation = new Translation2d(TRACK_WIDTH_METERS / 2.0, TRACK_LENGTH_METERS / 2.0);
    private final Translation2d m_frontRightLocation = new Translation2d(TRACK_WIDTH_METERS / 2.0, -TRACK_LENGTH_METERS / 2.0);
    private final Translation2d m_backLeftLocation = new Translation2d(-TRACK_WIDTH_METERS / 2.0, TRACK_LENGTH_METERS / 2.0);
    private final Translation2d m_backRightLocation = new Translation2d(-TRACK_WIDTH_METERS / 2.0, -TRACK_LENGTH_METERS / 2.0);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_backLeftLocation, m_frontRightLocation, m_backRightLocation);

//    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
//            m_kinematics,
//            ahrs.getRotation2d(),
//            new SwerveModulePosition[] {
//                comboFL.getPosition(),
//                comboBL.getPosition(),
//                comboFR.getPosition(),
//                comboBR.getPosition()
//            },
//            new Pose2d(5, 5, ahrs.getRotation2d()));

    private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
        m_kinematics,
        ahrs.getRotation2d(),
        new SwerveModulePosition[] {
                comboFL.getPosition(),
                comboBL.getPosition(),
                comboFR.getPosition(),
                comboBR.getPosition()
        },
        new Pose2d(5, 5, ahrs.getRotation2d()));

    Field2d m_field;

    public Chassis() {
        m_field = new Field2d();

        SmartDashboard.putData("Field", m_field);

        // TODO: Make sure this doesn't break anything
        ahrs.calibrate();
<<<<<<< Updated upstream
=======

        isLocked.setBoolean(false);

        if (IS_LOGGING) {
            DataLog log = DataLogManager.getLog();

            xPosLog = new DoubleLogEntry(log, "/my/xPos");
            yPosLog = new DoubleLogEntry(log, "/my/yPos");
            rotPosLog = new DoubleLogEntry(log, "/my/rotation");

            aprilXPosLog = new DoubleLogEntry(log, "/my/aprilXPos");
            aprilYPosLog = new DoubleLogEntry(log, "/my/aprilYPos");
            aprilRotPosLog = new DoubleLogEntry(log, "/my/aprilRotPos");

            deltaXPosLog = new DoubleLogEntry(log, "/my/deltaX");
            deltaYPosLog = new DoubleLogEntry(log, "/my/deltaY");
            deltaRotPosLog = new DoubleLogEntry(log, "/my/deltaAngle");
        }
>>>>>>> Stashed changes
    }

    // ported from last year
    public void runSwerve(double fwd, double str, double rot_temp) {

        // convenience for negating
        double rot = rot_temp;
        // sometimes happens if we align the modules up wrong, easier to just fix in here than redo

        // get new values
        new SwerveCalcs(fwd, str, rot);

        // driving ratio to tweak or go "Turbo Mode"
        double ratio = 1.0;

        if (isPrecision) {
            ratio = 0.15;
        }

        // assign new calc values
        double speedFL = 0;
        double speedBL = 0;
        double speedFR = 0;
        double speedBR = 0;

        try {
            speedFL = getSpeed(fwd, str, rot, 0);
            speedBL = getSpeed(fwd, str, rot, 1);
            speedFR = getSpeed(fwd, str, rot, 2);
            speedBR = getSpeed(fwd, str, rot, 3);
        } catch (Exception e) {
            e.printStackTrace();
        }

        // cap wheel speeds by finding max and adjusting all others down
        double maxWheelSpeed = max(max(speedFL, speedBL), max(speedFR, speedBR));

        // desaturate wheel speeds
        if (maxWheelSpeed > Constants.MAX_SPEED) {
            ratio = (Constants.MAX_SPEED/ maxWheelSpeed);
        }

        // pass all values to motors
        this.comboFL.passArgs(ratio * speedFL, getAngle(fwd, str, rot, 0));
        this.comboBL.passArgs(ratio * speedBL, getAngle(fwd, str, rot, 1));
        this.comboFR.passArgs(ratio * speedFR, getAngle(fwd, str, rot, 2));
        this.comboBR.passArgs(ratio * speedBR, getAngle(fwd, str, rot, 3));

    }

    // used for braking when scoring, balancing ideally
    public void crossWheels() {
        this.comboFL.passArgsNoDeadzone(0, -PI/4);
        this.comboBL.passArgsNoDeadzone(0, PI/4);
        this.comboFR.passArgsNoDeadzone(0, PI/4);
        this.comboBR.passArgsNoDeadzone(0, -PI/4);
    }



    // does the same calcs but passes a velocity of 0
    // used to align the modules in auton
    public void solveAngles(double fwd, double str, double rot) {

        new SwerveCalcs(fwd, str, rot);

        double ratio = 1.0;

        double speedFL = 0;
        double speedBL = 0;
        double speedFR = 0;
        double speedBR = 0;

        try {
            speedFL = getSpeed(fwd, str, rot, 0);
            speedBL = getSpeed(fwd, str, rot, 1);
            speedFR = getSpeed(fwd, str, rot, 2);
            speedBR = getSpeed(fwd, str, rot, 3);
        } catch (Exception e) {
            e.printStackTrace();
        }


        double maxWheelSpeed = max(max(speedFL, speedBL), max(speedFR, speedBR));

        if (maxWheelSpeed > Constants.MAX_SPEED) {
            ratio = (Constants.MAX_SPEED/ maxWheelSpeed);
        } else {
            ratio = 1.0;
        }


        this.comboFL.passArgs(0.03, getAngle(fwd, str, rot, 0));
        this.comboBL.passArgs(0.03, getAngle(fwd, str, rot, 1));
        this.comboFR.passArgs(0.03, getAngle(fwd, str, rot, 2));
        this.comboBR.passArgs(0.03, getAngle(fwd, str, rot, 3));

    }

    public void feedAll() {
        drive0.feed();
        drive1.feed();
        drive2.feed();
        drive3.feed();
        axis0.feed();
        axis1.feed();
        axis2.feed();
        axis3.feed();
    }

    @Override
    public void periodic() {

        double[] newPos = limelightField.getDoubleArray(new double[6]);


        m_odometry.update(ahrs.getRotation2d(),
                new SwerveModulePosition[] {
                        comboFL.getPosition(),
                        comboBL.getPosition(),
                        comboFR.getPosition(),
                        comboBR.getPosition()
                });
        Pose2d aprilTagFieldEstimate = null;
        if (newPos.length > 0) {
            aprilTagFieldEstimate = new Pose2d(newPos[0], newPos[1], new Rotation2d(newPos[5]));
            // resetOdometry(aprilTagFieldEstimate);
        }
        Pose2d stdFieldEstimate = m_odometry.getEstimatedPosition();

        m_field.setRobotPose(stdFieldEstimate);

        System.out.println(m_odometry.getEstimatedPosition().getX() + " meters");

//        System.out.println(m_odometry.getPoseMeters().getX() + " meters");

        feedAll();
<<<<<<< Updated upstream
=======

        if (IS_LOGGING) {
            xPosLog.append(m_odometry.getEstimatedPosition().getX());
            yPosLog.append(m_odometry.getEstimatedPosition().getY());
            rotPosLog.append(m_odometry.getEstimatedPosition().getRotation().getDegrees());

            if (aprilTagFieldEstimate != null) {
                double aprilX = aprilTagFieldEstimate.getX();
                double aprilY = aprilTagFieldEstimate.getY();
                double aprilRot = aprilTagFieldEstimate.getRotation().getDegrees();

                double deltaX = aprilX - stdFieldEstimate.getX();
                double deltaY = aprilY - stdFieldEstimate.getY();
                double deltaRot = aprilRot - stdFieldEstimate.getRotation().getDegrees();

                deltaXPosLog.append(deltaX);
                deltaYPosLog.append(deltaY);
                deltaRotPosLog.append(deltaRot);
            }
        }

>>>>>>> Stashed changes
    }

    public void resetOdometry(Pose2d pose) {

        m_odometry.resetPosition(ahrs.getRotation2d(), new SwerveModulePosition[] {
                comboFL.getPosition(),
                comboBL.getPosition(),
                comboFR.getPosition(),
                comboBR.getPosition()
        }, pose);
    }

    public Pose2d getPose() {
        return m_odometry.getEstimatedPosition();
    }

    public void setModuleStates(SwerveModuleState[] desired) {
        comboFL.passState(desired[0]);
        comboBL.passState(desired[1]);
        comboFR.passState(desired[2]);
        comboBR.passState(desired[3]);
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if(isFirstPath){
                        this.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        this::getPose, // Pose supplier
                        this.m_kinematics, // SwerveDriveKinematics
                        new PIDController(10, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                        new PIDController(10, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                        this::setModuleStates, // Module states consumer
                        this // Requires this drive subsystem
                )
        );

    }

    public double getPitch(){
        return ahrs.getPitch();
    }

    public double getRoll(){
        return ahrs.getRoll();
    }

    public void resetGyro() {
        ahrs.reset();
        ahrs.setAngleAdjustment(0);
    }

    public void setGyroAngle(double angle) {
        ahrs.reset();
        ahrs.setAngleAdjustment(angle);
    }

    public void togglePrecision() {
        isPrecision = !isPrecision;
    }

}
