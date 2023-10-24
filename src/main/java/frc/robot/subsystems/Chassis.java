package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.math.FusionSwerveOdometry;
import frc.robot.math.SwerveCalcs;
import frc.robot.math.SwerveConverter;
import frc.robot.modules.SwerveCombo;

import static frc.robot.Constants.*;
import static frc.robot.math.SwerveCalcs.getAngle;
import static frc.robot.math.SwerveCalcs.getSpeed;
import static java.lang.Double.max;
import static java.lang.Math.*;

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

    ShuffleboardTab tab = Shuffleboard.getTab("General");

    public GenericEntry isLocked = tab.add("Locked", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    public double desiredHeading = 0;

    public boolean isPrecision = false;

    DoubleLogEntry xPosLog;
    DoubleLogEntry yPosLog;
    DoubleLogEntry rotPosLog;


    // odometry set-up (import - Ri3D Redux)
    private final Translation2d m_frontLeftLocation = new Translation2d(TRACK_WIDTH_METERS / 2.0, TRACK_LENGTH_METERS / 2.0);
    private final Translation2d m_frontRightLocation = new Translation2d(TRACK_WIDTH_METERS / 2.0, -TRACK_LENGTH_METERS / 2.0);
    private final Translation2d m_backLeftLocation = new Translation2d(-TRACK_WIDTH_METERS / 2.0, TRACK_LENGTH_METERS / 2.0);
    private final Translation2d m_backRightLocation = new Translation2d(-TRACK_WIDTH_METERS / 2.0, -TRACK_LENGTH_METERS / 2.0);

    // need to see how much changing this affects auton
//    private final Translation2d m_frontLeftLocation = new Translation2d(TRACK_WIDTH_METERS_TEST / 2.0, TRACK_LENGTH_METERS_TEST / 2.0);
//    private final Translation2d m_frontRightLocation = new Translation2d(TRACK_WIDTH_METERS_TEST / 2.0, -TRACK_LENGTH_METERS_TEST / 2.0);
//    private final Translation2d m_backLeftLocation = new Translation2d(-TRACK_WIDTH_METERS_TEST / 2.0, TRACK_LENGTH_METERS_TEST / 2.0);
//    private final Translation2d m_backRightLocation = new Translation2d(-TRACK_WIDTH_METERS_TEST / 2.0, -TRACK_LENGTH_METERS_TEST / 2.0);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_backLeftLocation, m_frontRightLocation, m_backRightLocation);

    private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
            m_kinematics,
            ahrs.getRotation2d(),
            new SwerveModulePosition[] {
                comboFL.getPosition(),
                comboBL.getPosition(),
                comboFR.getPosition(),
                comboBR.getPosition()
            },
            new Pose2d(5, 5, ahrs.getRotation2d())
//            , VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(5)), // wheel odometry std filter // might be different
//            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    ); // vision std filter

    private final FusionSwerveOdometry m_odoTest = new FusionSwerveOdometry(5, 5, ahrs.getRotation2d(),
            new SwerveModulePosition[] {
                    comboFL.getPosition(),
                    comboBL.getPosition(),
                    comboFR.getPosition(),
                    comboBR.getPosition()
            });

    Field2d m_field;

    public Chassis() {
        m_field = new Field2d();

        SmartDashboard.putData("Field", m_field);

        // TODO: Make sure this doesn't break anything
        ahrs.calibrate();

        isLocked.setBoolean(false);

        if (IS_LOGGING) {
            DataLog log = DataLogManager.getLog();

            xPosLog = new DoubleLogEntry(log, "/my/xPos");
            yPosLog = new DoubleLogEntry(log, "/my/yPos");
            rotPosLog = new DoubleLogEntry(log, "/my/rotation");
        }
    }

    // ported from last year
    public void runSwerve(double fwd, double str, double rot_temp) {

        
        
        // convenience for negating
        double rot = rot_temp;
        /* 
        // sometimes happens if we align the modules up wrong, easier to just fix in here than redo

        // get new values
        new SwerveCalcs(fwd, str, rot);

        // driving ratio to tweak or go "Turbo Mode"
        double ratio = 1.0;

        // deprecated from old precision mode
        // new one takes place in command instead of here
//        if (isPrecision) {
//            ratio = 0.15;
//        }

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
        */
        

        // pass all values to motors

        ChassisSpeeds speed = new ChassisSpeeds(.1 * str * Constants.SWERVE_STRAFE_SPEED_MAX, .1* fwd * Constants.SWERVE_FORWARD_SPEED_MAX, .1*rot * ( Constants.SWERVE_ROT_SPEED_MAX));

        SwerveModuleState[] states = SwerveConverter.toSwerveModules(m_kinematics, speed);

        // FL, FR, BL, BR
        /*SwerveModuleState[] states = new SwerveModuleState[]{
            new SwerveModuleState(ratio * speedFR, new Rotation2d(getAngle(fwd, str, rot, 0))),
            new SwerveModuleState(ratio * speedFL, new Rotation2d(getAngle(fwd, str, rot, 2))),
            new SwerveModuleState(ratio * speedBL, new Rotation2d(getAngle(fwd, str, rot, 1))),
            new SwerveModuleState(ratio * speedBR, new Rotation2d(getAngle(fwd, str, rot, 3)))
        };*/

        // update swerve modules with 254's method
        //states = updateChassisSpeeds(states);

        this.comboFL.passArgs(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        this.comboBL.passArgs(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        this.comboFR.passArgs(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        this.comboBR.passArgs(states[3].speedMetersPerSecond, states[3].angle.getRadians());
        
    }

    // used for braking when scoring, balancing ideally
    public void crossWheels() {
        this.comboFL.passArgsNoDeadzone(0, -PI/4);
        this.comboBL.passArgsNoDeadzone(0, PI/4);
        this.comboFR.passArgsNoDeadzone(0, PI/4);
        this.comboBR.passArgsNoDeadzone(0, -PI/4);
    }

//    public void setLockedWheels(boolean locked){
//        if(locked){
//            crossWheels();
//        }
//    }



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
        m_odometry.update(ahrs.getRotation2d(),
                new SwerveModulePosition[] {
                        comboFL.getPosition(),
                        comboBL.getPosition(),
                        comboFR.getPosition(),
                        comboBR.getPosition()
                });

        m_odoTest.updateCustomOdo(ahrs.getRotation2d(),
                new SwerveModulePosition[] {
                        comboFL.getPosition(),
                        comboBL.getPosition(),
                        comboFR.getPosition(),
                        comboBR.getPosition()
                });

        // m_field.setRobotPose(m_odometry.getEstimatedPosition());
        m_field.setRobotPose(m_odoTest.getRobotPose());
        m_odoTest.pushModuleLocationsToField(m_field);

         // System.out.println("(" + m_odometry.getEstimatedPosition().getX() + "," + m_odometry.getEstimatedPosition().getY() + ")");
//        System.out.println("(" + m_odoTest.getRobotPose().getX() + "," + m_odoTest.getRobotPose().getY() + ")");

        feedAll();

        if (IS_LOGGING) {
            xPosLog.append(m_odometry.getEstimatedPosition().getX());
            yPosLog.append(m_odometry.getEstimatedPosition().getY());
            rotPosLog.append(m_odometry.getEstimatedPosition().getRotation().getDegrees());
        }

    }

    public void resetOdometry(Pose2d pose) {

        m_odometry.resetPosition(ahrs.getRotation2d(), new SwerveModulePosition[] {
                comboFL.getPosition(),
                comboBL.getPosition(),
                comboFR.getPosition(),
                comboBR.getPosition()
        }, pose);
    }

    public void resetOdometryCustom(Pose2d pose) {

        m_odoTest.resetPosition(ahrs.getRotation2d(), new SwerveModulePosition[] {
                comboFL.getPosition(),
                comboBL.getPosition(),
                comboFR.getPosition(),
                comboBR.getPosition()
        }, pose);
    }

    public Pose2d getPose() {
        return m_odometry.getEstimatedPosition();
    }
/*
 Pose2d robot_pose_vel = new Pose2d(mPeriodicIO.des_chassis_speeds.vxMetersPerSecond * Constants.kLooperDt,
            mPeriodicIO.des_chassis_speeds.vyMetersPerSecond * Constants.kLooperDt,
            Rotation2d.fromRadians(mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond * Constants.kLooperDt));
Twist2d twist_vel = Pose2d.log(robot_pose_vel);
ChassisSpeeds updated_chassis_speeds = new ChassisSpeeds(
            twist_vel.dx / Constants.kLooperDt, twist_vel.dy / Constants.kLooperDt, twist_vel.dtheta / Constants.kLooperDt);


// Getting individual speeds
double forward = chassisSpeeds.vxMetersPerSecond;
double sideways = chassisSpeeds.vyMetersPerSecond;
double angular = chassisSpeeds.omegaRadiansPerSecond;
 */
    // applying 254's method to update the odometry
    public SwerveModuleState[] updateChassisSpeeds(SwerveModuleState[] desired) {

        // get swerve module states
        SwerveModuleState flState = desired[0];
        SwerveModuleState frState = desired[1];
        SwerveModuleState blState = desired[2];
        SwerveModuleState brState = desired[3];

        // converts swerve module states to chassis speed
        ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(flState, frState, blState, brState);

        // delta velocity
        Pose2d robot_pose_vel = new Pose2d(chassisSpeeds.vxMetersPerSecond * Constants.kLooperDt,
        chassisSpeeds.vyMetersPerSecond * Constants.kLooperDt,
        Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * Constants.kLooperDt));
        Twist2d twist_vel = new Pose2d().log(robot_pose_vel);

        return m_kinematics.toSwerveModuleStates(new ChassisSpeeds(twist_vel.dx / kLooperDt, twist_vel.dy / kLooperDt, twist_vel.dtheta / kLooperDt));

    }

    public Pose2d getPoseCustom() {
        return m_odoTest.getRobotPose();
    }

    public void passVisionMeasurement(Pose2d measurement) {
        m_odometry.addVisionMeasurement(measurement, Timer.getFPGATimestamp());
    }

    public void setModuleStates(SwerveModuleState[] desired) {

        // update swerve module state with 254's method

        SwerveModuleState[] updatedDesired = updateChassisSpeeds(desired);
        // Front left module state
        SwerveModuleState frontLeft = updatedDesired[0];

        // Front right module state
        SwerveModuleState frontRight = updatedDesired[1];

        // Back left module state
        SwerveModuleState backLeft = updatedDesired[2];

        // Back right module state
        SwerveModuleState backRight = updatedDesired[3];

        comboFL.passState(frontLeft);
        comboBL.passState(backLeft);
        comboFR.passState(frontRight);
        comboBR.passState(backRight);
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
                        new PIDController(15, 0, 0.01), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                        new PIDController(15, 0, 0.01), // Y controller (usually the same values as X controller)
                        new PIDController(5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                        this::setModuleStates,
                        false, // Module states consumer
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
        desiredHeading = 0;
    }

    public void setGyroAngle(double angle) {
        ahrs.calibrate();
        ahrs.reset();
        ahrs.setAngleAdjustment(angle);
    }

    public void setPrecisionTrue(){
        isPrecision = true;
    }

    public void setPrecisionFalse(){
        isPrecision = false;
    }

    public void togglePrecision() {
        isPrecision = !isPrecision;
    }

    public void resetCustomOdoToOrigin() {
        m_odoTest.resetOdometryPoseOnly(new Pose2d(0, 0, ahrs.getRotation2d()));
    }

}
