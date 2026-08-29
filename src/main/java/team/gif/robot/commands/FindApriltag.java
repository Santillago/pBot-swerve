package team.gif.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Robot;

public class FindApriltag extends Command {

    double rpm;

    public FindApriltag() {
        super();
        addRequirements(Robot.motorControl);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        /*
        The PID controller calculates the difference between the current position of the motor and the
        desired setpoint and applies different gains to produce an output.
        Example output (P only): Output = Error * kP
        */
        rpm = Robot.motorControl.motorPIDController.calculate(Robot.motorLimelight.getXOffset(), 0.0);
        // Because PID outputs are often unpredictable, it's good practice to clamp the values to prevent danger.
        rpm = MathUtil.clamp(rpm, -75, 75);
        // Send the desired RPM control to the motor.
        Robot.motorControl.setRPM(rpm);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Always stop the motor at the end of a command to prevent accidents from occurring.
        Robot.motorControl.stopMotor();
    }
}
