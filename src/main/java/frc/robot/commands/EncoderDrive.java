// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.jsontype.impl.LaissezFaireSubTypeValidator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class EncoderDrive extends CommandBase {
  /** Creates a new EncoderDrive. */
  DriveTrain dt;
  Double setpoint;

  public EncoderDrive(DriveTrain dt, Double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies. we are passing
    // DriveTrain through
    this.dt = dt;
    this.setpoint = setpoint;
    addRequirements(dt);
  }

  // declare the subsystem
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetEncoders();
    
  }
  // calling a method from DriveTrain
// method= a function- an input and an output 
  @Override
  public void execute() {
      dt.tankDrive(0.2, 0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return dt.TicksToMeters() >= setpoint; 
  }
}
