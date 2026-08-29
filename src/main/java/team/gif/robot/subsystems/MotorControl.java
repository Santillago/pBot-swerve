// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class MotorControl extends SubsystemBase {

    private SparkMax neo;
    private SparkMaxConfig config;
    public PIDController motorPIDController;
    private double kP, kI, kD;

    public MotorControl() {
        /*
        Set the configuration for our SparkMax motor controller.
        This includes things such as our idle mode (what the motor does once it stops receiving power) and
        whether the motors values are inverted.
        */
        setConfig();
        // Initialize the PIDController, which helps the SparkMax send more accurate controls to the motor.
        motorPIDController = new PIDController(kP, kI, kD);
        // Add initial PID values to dashboard.
        SmartDashboard.putNumber("P", kP);
        SmartDashboard.putNumber("I", kI);
        SmartDashboard.putNumber("D", kD);
    }

    // An algorithm that detects a change in the PID values in the dashboard and updates accordingly.
    @Override
    public void periodic() {
        System.out.print("P " + kP);
        double nuP, nuI, nuD;
        nuP = SmartDashboard.getNumber("P", kP);
        nuI = SmartDashboard.getNumber("I", kI);
        nuD = SmartDashboard.getNumber("D", kD);
        if(nuP != kP || nuI != kI || nuD != kD) {
            kP = nuP; kI = nuI; kD = nuD;
            motorPIDController.setPID(kP, kI, kD);
        }
    }

    // A method that controls the motor based on the set velocity (in units of RPM).
    public void setRPM(double velocity) {
        neo.getClosedLoopController().setSetpoint(velocity, SparkMax.ControlType.kVelocity);
    }

    public void setVoltage(double v) {
        neo.setVoltage(v);
    }

    // A method that immediately stops the motor.
    public void stopMotor() {
        neo.stopMotor();
    }

    // A method that returns the current velocity of the motor as detected by a built-in encoder.
    public double getVelocity() {
        return neo.getEncoder().getVelocity();
    }

    public void setConfig() {
        neo = new SparkMax(RobotMap.SPARKMAX_ID, SparkLowLevel.MotorType.kBrushless);
        config = new SparkMaxConfig();
        config.idleMode(SparkMaxConfig.IdleMode.kCoast);
        config.inverted(false);
        neo.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

}
