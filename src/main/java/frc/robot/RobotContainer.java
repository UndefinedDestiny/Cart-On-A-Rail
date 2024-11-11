// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Auto;
import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final Command auto = new Auto();
  private final MotorSubsystem motorSubsystem = new MotorSubsystem();
  private final CommandXboxController driverController = new CommandXboxController(0);


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driverController.a().whileTrue(motorSubsystem.move(0.0));
  }

  public Command getAutonomousCommand() {
    return auto;
  }
}
