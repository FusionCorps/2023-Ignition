package frc.robot.commands.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chassis;

public class FourMetersForward extends SequentialCommandGroup {

    Chassis mChassis;

    PathPlannerTrajectory fourMetersForward;

    public FourMetersForward(Chassis chassis, boolean isRed) {
        mChassis = chassis;

        fourMetersForward = PathPlanner.loadPath("FourMetersRight", new PathConstraints(5,1));

        addRequirements(mChassis);

        if (isRed){
            fourMetersForward = PathPlannerTrajectory.transformTrajectoryForAlliance(fourMetersForward, DriverStation.Alliance.Red);
        }

        addCommands(
            mChassis.runOnce(() -> { mChassis.setGyroAngle(0.0); }),
            mChassis.followTrajectoryCommand(fourMetersForward, true)
        );

    }

}
