package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;

public class OnePieceTaxiBalance extends SequentialCommandGroup {

    Chassis m_chassis;
    Arm m_arm;
    Intake m_intake;
    Cameras m_camera;


}
