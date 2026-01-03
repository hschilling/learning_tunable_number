// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.TunableNumber;

public class ShooterSubsystem extends SubsystemBase {
  // Simulation objects
  private final FlywheelSim m_flywheelSim;

  private final LinearSystem<N1, N1, N1> m_plant;
  private final DCMotor m_gearbox = DCMotor.getNEO(1);

  // Feedforward controller (will be recreated when tunable values change)
  private SimpleMotorFeedforward m_feedforward;

  // PID Controller (P-only for feedback correction)
  private final PIDController m_pidController;

  // Tunable feedforward values
  private final TunableNumber m_kS;
  private final TunableNumber m_kV;

  // Tunable PID value
  private final TunableNumber m_kP;

  // Current setpoint in RPM
  private double m_targetRPM = 0.0;

  // Applied voltage
  private double m_appliedVoltage = 0.0;

  public ShooterSubsystem(){
  /** Creates a new ShooterSubsystem. */
  // IMPORTANT: Parameter order is (motor, MOI, gearing) - NOT (motor, gearing, MOI)!
  m_plant =
      LinearSystemId.createFlywheelSystem(m_gearbox, ShooterConstants.MOMENT_OF_INERTIA, ShooterConstants.GEAR_RATIO);

    m_flywheelSim = new FlywheelSim(m_plant, m_gearbox);

    // Initialize feedforward controller
    m_feedforward = new SimpleMotorFeedforward(ShooterConstants.FEEDFORWARD_S, ShooterConstants.FEEDFORWARD_V);

    // Initialize PID controller with P-only
    m_pidController = new PIDController(ShooterConstants.FEEDBACK_P, 0, 0);

    // Create tunable numbers for feedforward gains
    m_kS = new TunableNumber("Shooter/kS", ShooterConstants.FEEDFORWARD_S, Constants.TUNING_MODE);
    m_kV = new TunableNumber("Shooter/kV", ShooterConstants.FEEDFORWARD_V, Constants.TUNING_MODE);

    // Create tunable number for P gain
    m_kP = new TunableNumber("Shooter/kP", ShooterConstants.FEEDBACK_P, Constants.TUNING_MODE);
  }

  /**
   * Sets the target shooter speed in RPM.
   *
   * @param rpm The target speed in revolutions per minute
   */
  public void setTargetRPM(double rpm) {
    m_targetRPM = rpm;
  }

  /**
   * Gets the current shooter speed in RPM.
   *
   * @return The current speed in revolutions per minute
   */
  public double getCurrentRPM() {
    // Convert from radians per second to RPM
    return m_flywheelSim.getAngularVelocityRPM();
  }

  /**
   * Stops the shooter by setting target to 0 RPM.
   */
  public void stop() {
    m_targetRPM = 0.0;
    m_appliedVoltage = 0.0;
    m_flywheelSim.setInputVoltage(0.0);
  }

  /**
   * Command factory to run the shooter at a specific RPM.
   *
   * @param rpm The target speed in RPM
   * @return A command that runs the shooter at the specified speed
   */
  public Command runAtRPM(double rpm) {
    return run(() -> setTargetRPM(rpm))
        .finallyDo(() -> stop())
        .withName("RunShooter_" + rpm + "RPM");
  }

  @Override
  public void periodic() {
    // Update feedforward controller if gains changed in tuning mode
    // Note: SimpleMotorFeedforward is immutable, so we must recreate it when values change
    // Alternative approach (commented below): calculate feedforward manually using the formula
    if (m_kS.hasChanged() || m_kV.hasChanged()) {
      m_feedforward = new SimpleMotorFeedforward(m_kS.get(), m_kV.get());
    }

    // Update P gain if it changed in tuning mode
    if (m_kP.hasChanged()) {
      m_pidController.setP(m_kP.get());
    }

    // Get current velocity
    double currentRPM = getCurrentRPM();

    // Calculate feedforward voltage using the SimpleMotorFeedforward object
    // This demonstrates how to handle immutable controller objects with TunableNumbers
    double feedforwardVoltage = m_feedforward.calculate(m_targetRPM);

    // Alternative approach: Calculate feedforward manually (simpler for this case)
    // double feedforwardVoltage = m_kS.get() * Math.signum(m_targetRPM) + m_kV.get() * m_targetRPM;

    // Calculate feedback voltage (small correction for errors)
    double feedbackVoltage = m_pidController.calculate(currentRPM, m_targetRPM);

    // Combine feedforward and feedback
    m_appliedVoltage = feedforwardVoltage + feedbackVoltage;

    // Clamp voltage to reasonable limits (-12V to +12V)
    m_appliedVoltage = Math.max(-12.0, Math.min(12.0, m_appliedVoltage));

    // Apply voltage to simulation
    m_flywheelSim.setInputVoltage(m_appliedVoltage);

    // Update telemetry
    SmartDashboard.putNumber("Shooter/TargetRPM", m_targetRPM);
    SmartDashboard.putNumber("Shooter/CurrentRPM", currentRPM);
    SmartDashboard.putNumber("Shooter/FeedforwardVoltage", feedforwardVoltage);
    SmartDashboard.putNumber("Shooter/FeedbackVoltage", feedbackVoltage);
    SmartDashboard.putNumber("Shooter/AppliedVoltage", m_appliedVoltage);
    SmartDashboard.putNumber("Shooter/Error", m_targetRPM - currentRPM);
  }

  @Override
  public void simulationPeriodic() {
    // Update the simulation (called every 20ms)
    m_flywheelSim.update(0.02);
  }
}
