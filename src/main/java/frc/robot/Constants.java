// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;
  public static final XboxController controller = new XboxController(0);

  public static final double AMotorSpeed = 0.1;


  public static final class OperatorConstants {

    public static final int kDriverControllerPort = 0; // Update port number as needed

  }

  public static final class AMotorConstants {
    public static final double CURRENT_LIMIT_AMPS = 30.0;
    public static final double CURRENT_SPIKE_THRESHOLD = 40.0;
    public static final int MOTOR_UPDATE_FREQUENCY = 50;
    public static final double TICKS_PER_REVOLUTION = 2048.0;
    public static final double HUNDRED_MS_TO_SECONDS = 10.0;
    public static final double DEFAULT_VELOCITY = 2.0; // rotations per second
    public static final double DEFAULT_FEEDFORWARD = 0.1;
}



  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
