package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class Align extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Chassis m_chassis;
    PhotonCamera camera = new PhotonCamera("photonvision");
    PIDController turnController = new PIDController(Constants.Align_P, 0,Constants.Align_D);

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Align (Chassis subsystem) {
      m_chassis = subsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var rotationspeed = 0.0;
        var result = camera.getLatestResult();
        if (result.hasTargets()){
            rotationspeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
        }
        m_chassis.runSwerve(0, 0, rotationspeed);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }
  