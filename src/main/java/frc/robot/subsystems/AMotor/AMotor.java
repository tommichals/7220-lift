// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AMotor;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.AMotorConstants.*;

public class AMotor extends SubsystemBase {
  private final AMotorOTalonFX motorController;
  private final SimpleMotorFeedforward ffModel;

  /** Creates a new AMotor. */
  public AMotor(AMotorOTalonFX motorController) {
    this.motorController = motorController;
    ffModel = new SimpleMotorFeedforward(0.1, 0.05);
    motorController.configurePID(1.0, 1.0, 0.0);
  }

  public void runMotorToEncoderValue(double rotations) {
    motorController.runMotorToEncoderValue(rotations);
  }

  @Override
  public void periodic() {
    double rotations = motorController.getMotorRotations();
    System.out.println("AMotor Periodic - Motor Rotations: " + rotations);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double voltage) {
    motorController.setVoltage(voltage);
  }

  /** Run closed loop at the specified speed. */
  public void runVelocity(double rotationsPerSecond) {
    motorController.setVelocity(rotationsPerSecond, ffModel.calculate(rotationsPerSecond));
}

  /** Stops the motor. */
  public void stop() {
    motorController.stop();
  }

  /** Returns the current speed. */
  public double getSpeed() {
    return Units.feetToMeters(motorController.getSpeed());
  }

  /** Returns the current speed for characterization. */
  public double getCharacterizationSpeed() {
    return motorController.getSpeed();
  }
}
