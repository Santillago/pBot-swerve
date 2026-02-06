// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team.gif.robot.RobotMap;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Collector extends SubsystemBase {

    public TalonFX collector;
    public TalonFXConfiguration config = new TalonFXConfiguration();
    public VelocityVoltage velocityVoltage;

    public Collector() {
        collector = new TalonFX(RobotMap.COLLECTOR_ID);
//        config.Slot0.kP = 0;
//        config.Slot0.kI = 0;
//        config.Slot0.kD = 0;

        setConfig(config);

        velocityVoltage = new VelocityVoltage(0).withSlot(0);
    }

    /**
    @Override
    public void periodic() {

        double netP = SmartDashboard.getNumber("PID/P", 0);
        double netI = SmartDashboard.getNumber("PID/I", 0);
        double netD = SmartDashboard.getNumber("PID/D", 0);

        double currP = config.Slot0.kP;
        double currI = config.Slot0.kI;
        double currD = config.Slot0.kD;


        if(netP != currP || netI != currI || netD != currD) {
            config.Slot0.kP = netP;
            config.Slot0.kI = netI;
            config.Slot0.kD = netD;
            config.Slot0.kS = 0.12278;
            config.Slot0.kV = 0.11522;
            config.Slot0.kA = 0.0078728;
            setConfig(config);
        }

    }
    **/

    public void runShooter(double rpm) {
        collector.setControl(velocityVoltage.withVelocity(-rpm/60));
    }

    public void runVoltage(double voltage){
        collector.setVoltage(voltage);
    }

    public double getVelocity() {
        return collector.getBridgeOutput().getValueAsDouble();
    }

    public double getVoltage(){
        return collector.getMotorVoltage().getValueAsDouble();
    }

    public void stopMotor() {
        collector.stopMotor();
    }

    public void setConfig(TalonFXConfiguration config) {
        collector.getConfigurator().apply(config);
    }

    private void sysIDVoltage(Voltage volt) {
        runVoltage(volt.baseUnitMagnitude());
    }

    private void sysIDLog(SysIdRoutineLog log) {
        MutVoltage voltMut = Volts.mutable(0);
        MutAngle posMut = Rotations.mutable(0);
        MutAngularVelocity vMut= RotationsPerSecond.mutable(0);

        log.motor("Shooter")
                .voltage(voltMut.mut_replace(getVoltage(), Volts))
                .angularVelocity(vMut.mut_replace(collector.getVelocity().getValueAsDouble(), RotationsPerSecond))
                .angularPosition(posMut.mut_replace(collector.getPosition().getValueAsDouble(), Rotations));
    }


    public SysIdRoutine getSysID() {
        return new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(this::sysIDVoltage, this::sysIDLog, this)
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return getSysID().quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return getSysID().dynamic(direction);
    }



}
