// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors. CANCoder; 
import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame; 
import com.revrobotics.RelativeEncoder; 
import com.revrobotics.SparkMaxPIDController; 

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleIOSparkMax implements SwerveModuleIO{
    private CANSparkMax driveSparkMax; 
    private CANSparkMax turnSparkMax; 
    private SparkMaxPIDController drivePID; 
    private PIDController turnPID; 
    private RelativeEncoder driveEncoder; 
    private CANCoder turnEncoder; 

    private SwerveModuleState state = new SwerveModuleState(0.0, new Rotation2d(0.0)); 
    private int num =0; 

    
    public SwerveModuleIOSparkMax(int num, int driveID, int turnID, int turnCANCoderID, double turnEncoderOffset){
        turnEncoder = new CANCoder(turnCANCoderID); 

        // CONSTRUST CANSparkMaxes
        driveSparkMax = new CANSparkMax(driveID, MotorType.kBrushless); 
        turnSparkMax = new CANSparkMax(turnID, MotorType.kBrushless); 
        turnPID = new PIDController(0, 0, 0); 

        //INITIALISE ENCODER AND PID controller
        driveEncoder = driveSparkMax.getEncoder(); 
        drivePID = driveSparkMax.getPIDController();
        drivePID.setFeedbackDevice(driveEncoder); 

        driveSparkMax.restoreFactoryDefaults(); 
        turnSparkMax.restoreFactoryDefaults(); 

        driveEncoder.setPositionConversionFactor(1.0);
        driveEncoder.setVelocityConversionFactor(1.0); 

        //set Spark MAX PIDF
        //more advantageous due to 1KHz cycle (can ramp up action quicker)
        drivePID.setP(0); 
        drivePID.setI(0);
        drivePID.setD(0);
        drivePID.setFF(0);
        drivePID.setOutputRange(-1,1); 

        driveSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turnSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveSparkMax.setSmartCurrentLimit(50);
        turnSparkMax.setSmartCurrentLimit(20); 

        driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); 
        turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); 

        driveEncoder.setPosition(0); 

        //set position of encoder to absolute mode
        turnEncoder.setPositionToAbsolute(); 
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); 
        turnEncoder.configMagnetOffset(turnEncoderOffset); 

        turnPID.setP(0);
        turnPID.setI(0);
        turnPID.setD(0);

        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        this.state.angle = new Rotation2d(getTurnPositionInRad()); 

        this.num = num; 

    }

    public double getTurnPositionInRad(){
        //divide by 1.0, as CANCoder has direct measuremnt of output
        return((turnEncoder.getAbsolutePosition()/4096.0)*2*Math.PI); 
    }





}