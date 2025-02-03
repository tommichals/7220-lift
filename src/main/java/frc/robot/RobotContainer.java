// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AMotorConstants;


import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.AMotor.AMotor;
import frc.robot.subsystems.AMotor.AMotorOTalonFX;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController controller = new CommandXboxController(0);

 // Subsystems


   // Controller and Motor
    private final AMotor aMotor = new AMotor(new AMotorOTalonFX(9));

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

 // Register commands
// The following line is commented out because it is part of PathPlanner functionality:
// NamedCommands.registerCommand("Run AMotor", new AMotorRun(aMotor));

 // Set up auto routines
//tom    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());



    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
 controller
    .rightBumper()
    .whileTrue(
        Commands.startEnd(
            () -> {
                System.out.println("Right Bumper: Forward " + AMotorConstants.DEFAULT_VELOCITY + " RPS");
                aMotor.runVelocity(AMotorConstants.DEFAULT_VELOCITY);
            },
            () -> {
                System.out.println("Right Bumper Released: Stopping");
                aMotor.stop();
            },
            aMotor));

controller
    .leftBumper()
    .whileTrue(
        Commands.startEnd(
            () -> {
                System.out.println("Left Bumper: Reverse " + -AMotorConstants.DEFAULT_VELOCITY + " RPS");
                aMotor.runVelocity(-AMotorConstants.DEFAULT_VELOCITY);
            },
            () -> {
                System.out.println("Left Bumper Released: Stopping");
                aMotor.stop();
            },
            aMotor));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
