// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.gif.robot.RobotMap;

public class DiffDrive extends SubsystemBase {

    private WPI_TalonSRX leftMotor;
    private WPI_TalonSRX rightMotor;
    private DifferentialDrive differentialDrive;

    public DiffDrive() {
        leftMotor = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_TALON_ID);
        rightMotor = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_TALON_ID);

        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);

        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();

        differentialDrive = new DifferentialDrive(leftMotor, rightMotor);

    }

    public void arcadeDrive(double speed, double turn){
        differentialDrive.arcadeDrive(speed, turn);
    }

}
