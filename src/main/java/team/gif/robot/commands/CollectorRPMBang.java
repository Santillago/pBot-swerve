package team.gif.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class CollectorRPMBang extends Command {

    private double rpm;
    private final BangBangController bangBangController;

    public CollectorRPMBang() {
        super();
        addRequirements(Robot.collector);
        this.bangBangController = new BangBangController();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rpm = SmartDashboard.getNumber("Collector/RPM", 0);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double bangOutput = bangBangController.calculate(Robot.collector.getVelocity(), rpm);
        double motorVoltage = bangOutput * Constants.Collector.COLLECTOR_FF.calculate(rpm);
        Robot.collector.runVoltage(motorVoltage);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.collector.stopMotor();
    }
}
