// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Global tuning mode - set to true to enable live tuning via SmartDashboard
  public static final boolean TUNING_MODE = false;

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static class ShooterConstants {
    // Feedforward constants (for SimpleMotorFeedforward)
    public static final double FEEDFORWARD_S = 0.25;  // Voltage to overcome static friction
    public static final double FEEDFORWARD_V = 0.00211;  // Voltage per RPM (12V / 5676 RPM for NEO)

    // PID constants (P-only feedback control)
    public static final double FEEDBACK_P = 0.004;  // P gain to provide acceleration voltage during large errors

    // Physical properties for simulation
    public static final double GEAR_RATIO = 1.0;  // Direct drive
    public static final double MOMENT_OF_INERTIA = 0.004;  // kg*m^2, typical flywheel for shooter

    // Target speeds in RPM
    public static final double LOW_SPEED_RPM = 2000.0;   // Low speed shot
    public static final double MID_SPEED_RPM = 3500.0;   // Medium speed shot
    public static final double HIGH_SPEED_RPM = 5000.0;  // High speed shot
  }
}
