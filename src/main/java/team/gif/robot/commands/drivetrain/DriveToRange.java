package team.gif.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class DriveToRange extends Command {

    private PIDController distancePID;

    public DriveToRange() {
        super();
        this.distancePID = new PIDController(Constants.Limelight.TELE_APRILTAG_DISTANCE_P, 0, 0);
        addRequirements(Robot.swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        double targetOffsetAngle_Y = Robot.limelight.getYOffset();
        double goalAngle = Constants.Limelight.LIMELIGHT_MOUNT_ANGLE_DEG + targetOffsetAngle_Y;
        double goalRadians = Math.toRadians(goalAngle);
        double currentDistance_In = (Constants.Limelight.LIMELIGHT_GOAL_HEIGHT_INCHES - Constants.Limelight.LIMELIGHT_LENS_HEIGHT_INCHES) / Math.tan(goalRadians);

        double speed = distancePID.calculate(currentDistance_In+Constants.Limelight.DESIRED_DISTANCE_IN, 0);
        speed *= Robot.swerveConfig.constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        speed *=-1.0;

        SmartDashboard.putNumber("Current Dist", currentDistance_In);

        Robot.swerveDrive.drive(speed, 0, 0);

    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}
