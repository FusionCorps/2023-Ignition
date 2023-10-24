package frc.robot.math;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// converts a ChassisSpeeds to an array of SwerveModuleStates
public class SwerveConverter {
    public static SwerveModuleState[] toSwerveModules(SwerveDriveKinematics kinematics, ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        
        // adds 90 degrees to all the s 
        for (int i = 0; i < states.length; i++) {
            states[i].angle = new Rotation2d(states[i].angle.getRadians() - Math.PI / 2);
        }
        return states;
    }
}
