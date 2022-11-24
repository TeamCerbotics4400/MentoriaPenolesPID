// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DrivePIDWPILib extends CommandBase {
  /** Creates a new DrivePIDWPILib. */
  Drivetrain drivetrain;
  double setpoint;

  PIDController controller = new PIDController(Constants.kP, Constants.kI, Constants.kD);

  public DrivePIDWPILib(Drivetrain drivetrain, double setpoint) {
    this.drivetrain = drivetrain;
    this.setpoint = setpoint;

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(setpoint);
    controller.setTolerance(0.01);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double salida = controller.calculate(drivetrain.getDistance());
    drivetrain.drive(salida, 0);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
