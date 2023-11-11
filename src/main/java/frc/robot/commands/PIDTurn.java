// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainPorts;
import frc.robot.subsystems.DriveTrain;

public class PIDTurn extends CommandBase {
  DriveTrain dt;
  double setpointAngle; 
  PIDController PID = new PIDController(DriveTrainPorts.kP, 0, 0);
  double motorSign; 
  
  //you are taking the "PID" object into the PIDController class
  /** Creates a new PIDTurn. */
  //to get the P contsant you need to divide moto power/error  
  public PIDTurn(DriveTrain dt, Double setpointAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    this.setpointAngle = setpointAngle;

    addRequirements(dt);
    PID.setTolerance(5.0);
    if (setpointAngle > 0 ){
      motorSign = 1.0; //counterclockwise turn 
    } else {
      motorSign = -1.0;  //clockwise turn
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   dt.resetNavx(); //Navx is a sensor gets an angle 
   dt.tankDrive(0, 0);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = PID.calculate(dt.getAngle(), setpointAngle);
    dt.tankDrive(-motorSign*output, motorSign*output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0, 0); //sets the motors back to 0
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PID.atSetpoint(); 
  }
}
