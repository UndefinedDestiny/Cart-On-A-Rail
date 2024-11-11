// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class MotorSubsystem extends SubsystemBase {
  private final TalonFX motor = new TalonFX(0);
  private final TalonFXSimState motorSimState = new TalonFXSimState(motor);
  private final double gearRatio = 1.0;
  //in cm
  private final double wheelRadius = 500.0;
  private final DCMotorSim motorSim = new DCMotorSim(DCMotor.getKrakenX60(1), gearRatio, 0.01);

  private DoublePublisher velo;
  private DoublePublisher posi;

  public MotorSubsystem() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");

    velo = table.getDoubleTopic("Velocity").publish();
    posi = table.getDoubleTopic("Position").publish();

  }

  public Command move(double voltage) {
    return startEnd(
      () -> {
        motor.setVoltage(voltage);
      },
      () -> {
        motor.setVoltage(0.0);
      });
  }

  public Command moveLeft(double leftXAxis, double voltage) {
    return startEnd(
      () -> {
        System.out.println(leftXAxis);
      }, 
      () -> {

      });
  }

  public Command moveRight(double rightXAxis, double voltage) {
    return startEnd(
      () -> {
        System.out.println(rightXAxis);
      }, 
      () -> {
        
      });
  }

  public void initSendable() {
    double pos = (motorSim.getAngularPositionRotations() * 2.0 * Math.PI * wheelRadius / gearRatio);
    double vel = (motorSim.getAngularVelocityRPM() * 2.0 * Math.PI * wheelRadius / gearRatio);
    posi.set(pos);
    velo.set(vel);
  }

  @Override
  public void periodic() {
    initSendable();
  }

  @Override
  public void simulationPeriodic() {
    motorSim.setInputVoltage(motorSimState.getMotorVoltage());
    motorSim.update(0.02);
    motorSimState.setRawRotorPosition(motorSim.getAngularPositionRotations());
    motorSimState.setRotorVelocity(motorSim.getAngularVelocityRPM() / 60);
  }
}