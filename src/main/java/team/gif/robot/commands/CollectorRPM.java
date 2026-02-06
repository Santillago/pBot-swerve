package team.gif.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class CollectorRPM extends Command {

    double desiredRPM = 0;
    private final BangBangController bangController;



    public CollectorRPM() {
        super();
        addRequirements(Robot.collector);
        this.bangController = new BangBangController();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        desiredRPM = SmartDashboard.getNumber("Collector/RPM", 0);
        desiredRPM = bangController.calculate(Robot.collector.getVelocity(), desiredRPM);
        Robot.collector.runShooter(desiredRPM);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {}

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
