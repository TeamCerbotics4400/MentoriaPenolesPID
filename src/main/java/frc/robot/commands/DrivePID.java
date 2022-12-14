// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DrivePID extends CommandBase {
  /** Creates a new DrivePID. */
  Drivetrain drivetrain;
  double setpoint;

  double lastError = 0, deltaT = 0, ultimoTiempo = 0, errorSum = 0;
  public DrivePID(Drivetrain drivetrain, double setpoint) {
    this.drivetrain = drivetrain;
    this.setpoint = setpoint;

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tiempoActual = Timer.getFPGATimestamp();
    deltaT = tiempoActual - ultimoTiempo;
    double error = setpoint - drivetrain.getDistance();
    errorSum = errorSum + error;
    double derivativa = (error - lastError) / deltaT;

    double salida = (Constants.kP * error) + (Constants.kD * derivativa) + (Constants.kI * errorSum);

    drivetrain.drive(salida, 0);


    lastError = error;
    ultimoTiempo = tiempoActual;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
