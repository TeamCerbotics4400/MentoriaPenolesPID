// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  CANSparkMax leftMaster = new CANSparkMax(Constants.leftMasterID, MotorType.kBrushless);
  CANSparkMax leftSlave = new CANSparkMax(Constants.leftSlaveID, MotorType.kBrushless);
  CANSparkMax rightMaster = new CANSparkMax(Constants.rightMasterID, MotorType.kBrushless);
  CANSparkMax rightSlave = new CANSparkMax(Constants.rightSlaveID, MotorType.kBrushless);

  MotorControllerGroup leftControllers = new MotorControllerGroup(leftMaster, leftSlave);
  MotorControllerGroup rightControllers = new MotorControllerGroup(rightMaster, rightSlave);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftControllers, rightControllers);

  RelativeEncoder encoderIzq = leftMaster.getEncoder();
  RelativeEncoder encoderDer = rightMaster.getEncoder();


  public Drivetrain() {
    rightMaster.setInverted(true);
    rightSlave.setInverted(true);

    
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setIdleMode(IdleMode.kBrake);
    leftSlave.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave.setIdleMode(IdleMode.kBrake);

    encoderIzq.setPosition(0);
    encoderDer.setPosition(0);

  }

  public void drive(double speed, double turn){
    differentialDrive.arcadeDrive(speed, turn);
  }

  public double encoderCountsToMeters(double encoderCounts){
    double wheelRotations = encoderCounts / 10.75;
    double distance = wheelRotations * (Math.PI * 0.1524);
    return distance;
  }

  public double getDistance(){
    return (encoderCountsToMeters(encoderIzq.getPosition()) + 
          encoderCountsToMeters(encoderDer.getPosition())) / 2;
  }

  public void resetSensors(){
    encoderDer.setPosition(0);
    encoderIzq.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }

  public void log(){
    
  }
}
