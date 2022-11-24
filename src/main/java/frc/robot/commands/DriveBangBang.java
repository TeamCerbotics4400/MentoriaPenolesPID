// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveBangBang extends CommandBase {
  /** Creates a new DriveBangBang. */
  Drivetrain drivetrain;
  double setpoint;
  public DriveBangBang(Drivetrain drivetrain, double setpoint) {
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
    if(drivetrain.getDistance() < setpoint){
      drivetrain.drive(1, 0);
    }
    else{
      drivetrain.drive(0, 0);
    }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getDistance() > setpoint;
  }
}
