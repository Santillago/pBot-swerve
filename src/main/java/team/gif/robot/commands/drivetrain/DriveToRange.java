package team.gif.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class DriveToRange extends Command {

    private final PIDController distancePID;
    public double currentDistance_In;

    public DriveToRange() {
        super();
        this.distancePID = new PIDController(Constants.Limelight.TELE_APRILTAG_DISTANCE_P, 0, 0);
        Robot.limelight.setDistanceEstimatorParams(Constants.Limelight.LIMELIGHT_MOUNT_ANGLE_DEG, Constants.Limelight.LIMELIGHT_LENS_HEIGHT_INCHES, Constants.Limelight.LIMELIGHT_GOAL_HEIGHT_INCHES, 0);
        addRequirements(Robot.swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putNumber("Limelight/Current_Dist", currentDistance_In);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        //Get distance using the getDistance() method from 0the limelight
        currentDistance_In = Robot.limelight.getDistance();

        /*
          Outputs P*error + I*Integral + D*Derivative
          Error = Measurement (1) - Setpoint (2)
          Output becomes 0 as the measurement approaches the setpoint
         */
        double speed = distancePID.calculate(currentDistance_In, Constants.Limelight.TELE_APRILTAG_DESIRED_DISTANCE_INCHES);
        speed = MathUtil.clamp(speed, -.5, .5);
        speed *= Robot.swerveConfig.constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;


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
