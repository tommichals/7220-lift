// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AMotor.AMotor;

public class AMotorRun extends Command {

  private final AMotor aMotor;

  /** Creates a new RunMotorCommand. */
  public AMotorRun(AMotor aMotor) {
    this.aMotor = aMotor;
    addRequirements(this.aMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //aMotor.runVelocity(Constants.AMotorSpeed * (RobotContainer.controller.getAButton() ? 1 : 0));
    aMotor.runVelocity(Constants.AMotorSpeed * (Constants.controller.getAButton() ? 1 : 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    aMotor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
