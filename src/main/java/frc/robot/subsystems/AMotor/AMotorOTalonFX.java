package frc.robot.subsystems.AMotor;

import frc.robot.Constants;
import frc.robot.Constants.AMotorConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.Queue;

public class AMotorOTalonFX extends SubsystemBase implements AMotorIO {
  private final TalonFX motor;
  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final Queue<Angle> motorPositionQueue;
  private final PIDController pidController;

  public AMotorOTalonFX(int canID) {
    motor = new TalonFX(9);
    motorPosition = motor.getPosition();
    
    motorVelocity = motor.getVelocity();
    
    motorAppliedVolts = motor.getMotorVoltage();
    
    motorCurrent = motor.getSupplyCurrent();

    motorPositionQueue = new LinkedList<>();
    
    pidController = new PIDController(0.1, 0.0, 0.0); // Initialize with default PID values

    System.out.println("**************FIRST configureMotor() method invoked.");
    configureMotor();

    // Set the update frequency for the motor position signal
    BaseStatusSignal.setUpdateFrequencyForAll(AMotorConstants.MOTOR_UPDATE_FREQUENCY, motorPosition);
  }

  @Override
  public void configurePID(double p, double i, double d) {
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
  }

  @Override
  public void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 0.1; // Set your PID constants here
    slot0.kI = 0.0;
    slot0.kD = 0.0;
    slot0.kV = 0.0; // Replace with the correct field name

    config.Slot0 = slot0;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motor.getConfigurator().apply(config);
    motor.optimizeBusUtilization();

    System.out.println(
        "AMotorOTalonFX - Motor configured with FeedbackDevice: "
            + config.Feedback.FeedbackSensorSource.toString());
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }


    @Override
    public void setVelocity(double speed, double feedforward) {
        motor.setControl(new VelocityVoltage(speed).withFeedForward(feedforward));
    }

  @Override
  public void stop() {
    motor.setControl(new VoltageOut(0));
  }

  @Override
    public double getMotorRotations() {
      double currentTicks = motor.getRotorPosition().getValueAsDouble();
    return currentTicks / AMotorConstants.TICKS_PER_REVOLUTION;
    }

  @Override
  public double getSpeed() {
    double ticksPer100ms = motor.getRotorVelocity().getValueAsDouble();
    return (ticksPer100ms * AMotorConstants.HUNDRED_MS_TO_SECONDS) / AMotorConstants.TICKS_PER_REVOLUTION;
  }

  @Override
  public void runMotorToEncoderValue(double rotations) {
    // Implement the method to run motor to a specific encoder value
  }

  // Example method to use the fields
  public void updateMotorStatus() {
    motorPositionQueue.add(motorPosition.getValue());
    System.out.println("Motor Position: " + motorPosition.getValue());
    System.out.println("Motor Velocity: " + motorVelocity.getValue());
    System.out.println("Motor Applied Volts: " + motorAppliedVolts.getValue());
    System.out.println("Motor Current: " + motorCurrent.getValue());
  }

  public boolean isCurrentSpike() {
      double currentAmps = motorCurrent.getValueAsDouble();
      return currentAmps >  AMotorConstants.CURRENT_SPIKE_THRESHOLD;
  }

  @Override
  public void periodic() {
      if (isCurrentSpike()) {
          stop();
          System.out.println("Current spike detected! Motor stopped.");
      }
  }



}
