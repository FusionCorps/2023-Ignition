package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public abstract class AllianceCommand extends SequentialCommandGroup {
    protected boolean isRed;
    public abstract AllianceCommand changeAlliance(boolean isRed);
}
