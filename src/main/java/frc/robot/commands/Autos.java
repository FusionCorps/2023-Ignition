// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//This file should contain auton commands.

package frc.robot.commands;

import frc.robot.commands.chassis.ChassisDriveAuton;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {

  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }
  
  public static CommandBase AutonLine(Chassis chassis) {
    return Commands.sequence(new ChassisDriveAuton(chassis, 0.3, 0.0, 0.0, 0.5));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
