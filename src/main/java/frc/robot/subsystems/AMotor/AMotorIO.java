package frc.robot.subsystems.AMotor;

import edu.wpi.first.units.measure.Angle;

public interface AMotorIO {
  void configurePID(double p, double i, double d);

  void configureMotor();

  void setVoltage(double volts);

  void setVelocity(double speed, double feedforward);

  void stop();

  double getMotorRotations();

  double getSpeed();

  void runMotorToEncoderValue(double rotations);
}
