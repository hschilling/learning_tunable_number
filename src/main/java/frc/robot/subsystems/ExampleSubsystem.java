// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;

public class ExampleSubsystem extends SubsystemBase {

  private final TunableNumber setpoint;

    /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    setpoint = new TunableNumber("Elevator Setpoint", 10, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("setPoint " +  setpoint.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

 /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

}



// if (Constants.tuningMode) {
//   pidController.setP(kP.get());
//   pidController.setI(kI.get());
//   pidController.setD(kD.get());
//   SmartDashboard.putNumber("Roller/VelocityRPM", masterEncoder.getVelocity());
//   SmartDashboard.putNumber("Roller/VelocityRPMFollower", followerEncoder.getVelocity());
//   SmartDashboard.putNumber("Roller/MasterCurrent", rollerMaster.getOutputCurrent());
//   SmartDashboard.putNumber("Roller/FollowerCurrent", rollerFollower.getOutputCurrent());
// }

