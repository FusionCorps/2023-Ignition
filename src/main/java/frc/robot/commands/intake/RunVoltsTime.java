package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunVoltsTime extends CommandBase {

    Intake mIntake;

    double volts;
    double time;

    Timer timer = new Timer();

    public RunVoltsTime(Intake intake, double voltage, double runTime) {
        mIntake = intake;

        volts = voltage;
        time = runTime;

        addRequirements(mIntake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        mIntake.setVolts(volts);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }

    @Override
    public void end(boolean isFinished) {
        mIntake.set(0);
    }

}
