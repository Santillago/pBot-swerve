package team.gif.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class DriveSwerve extends Command {
    private final SlewRateLimiter forwardLimiter;
    private final SlewRateLimiter strafeLimiter;
    private final SlewRateLimiter turnLimiter;
    private final PIDController alignPID, distancePID;

    public DriveSwerve() {
        this.forwardLimiter = new SlewRateLimiter(Robot.swerveConfig.constants.MAX_ACCEL_METERS_PER_SECOND_SQUARED);
        this.strafeLimiter = new SlewRateLimiter(Robot.swerveConfig.constants.MAX_ACCEL_METERS_PER_SECOND_SQUARED);
        this.turnLimiter = new SlewRateLimiter(Robot.swerveConfig.constants.MAX_ANGULAR_ACCEL_RADIANS_PER_SECOND_SQUARED);
        this.alignPID = new PIDController(Constants.Limelight.TELE_APRILTAG_ALIGNMENT_P, 0, 0);
        this.distancePID = new PIDController(Constants.Limelight.TELE_APRILTAG_DISTANCE_P, Constants.Limelight.TELE_APRILTAG_DISTANCE_I, Constants.Limelight.TELE_APRILTAG_DISTANCE_D);
        addRequirements(Robot.swerveDrive);
    }

    @Override
    public void initialize() {
        alignPID.setTolerance(Constants.Limelight.TELE_APRILTAG_ALIGN_MODE_ALIGN_TOLERANCE_DEGREES);
        distancePID.setTolerance(Constants.Limelight.TELE_APRILTAG_ALIGN_MODE_DISTANCE_TOLERANCE_INCHES);
    }

    @Override
    public void execute() {
            double forwardSign;
            double strafeSign;
            double strafe;

            if(Robot.swerveDrive.alignMode&&Robot.limelight.hasTarget()){
                double currentDistance_In = Robot.limelight.getDistance();
                double forward = !distancePID.atSetpoint() ? distancePID.calculate(currentDistance_In, Constants.Limelight.TELE_APRILTAG_DESIRED_DISTANCE_INCHES) : 0.0;
                forward = MathUtil.clamp(forward, -.5, .5);
                forward *= Robot.swerveConfig.constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
                forward *= -1.0;

                strafe = -Robot.oi.driver.getLeftX(); // need to invert because -X is left, +X is right
                strafe = (Math.abs(strafe) > Constants.Joystick.DEADBAND) ? strafe : 0.0;
//                strafeSign = strafe/Math.abs(strafe);
                strafe = Math.abs(strafe) * strafe;
//                strafe = .5 * Math.sqrt(2 - forward*forward + strafe*strafe + 2*strafe*Math.sqrt(2)) -
//                        .5 * Math.sqrt(2 - forward*forward + strafe*strafe - 2*strafe*Math.sqrt(2));
//                if( Double.isNaN(strafe) )
//                    strafe = strafeSign;
                strafe = strafeLimiter.calculate(strafe) * Robot.swerveDrive.getDrivePace().getValue();
                
                double targetRot = !alignPID.atSetpoint() ?  alignPID.calculate(Robot.limelight.getXOffset(), 0) : 0.0;
                targetRot = MathUtil.clamp(targetRot, -0.5, 0.5);
                targetRot *= Robot.swerveConfig.constants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
                targetRot *= Constants.Limelight.TELE_APRILTAG_ALIGN_BOOST_PERCENT;


                //Forward speed, Sideways speed, Rotation Speed
                Robot.swerveDrive.drive(forward, strafe*.3, targetRot);
            }
            else{
                double forward = -Robot.oi.driver.getLeftY(); // need to invert because -Y is away, +Y is pull back
                forward = (Math.abs(forward) > Constants.Joystick.DEADBAND) ? forward : 0.0; //0.00001;
                strafe = -Robot.oi.driver.getLeftX(); // need to invert because -X is left, +X is right
                strafe = (Math.abs(strafe) > Constants.Joystick.DEADBAND) ? strafe : 0.0;

                forwardSign = forward/Math.abs(forward);
                strafeSign = strafe/Math.abs(strafe);
                // Use a parabolic curve (instead if linear) for the joystick to speed ratio
                // This allows for small joystick inputs to use slower speeds
                forward = Math.abs(forward) * forward;
                strafe = Math.abs(strafe) * strafe;

                forward = .5 * Math.sqrt(2 + forward*forward - strafe*strafe + 2*forward*Math.sqrt(2)) -
                        .5 * Math.sqrt(2 + forward*forward - strafe*strafe - 2*forward*Math.sqrt(2));

                strafe = .5 * Math.sqrt(2 - forward*forward + strafe*strafe + 2*strafe*Math.sqrt(2)) -
                        .5 * Math.sqrt(2 - forward*forward + strafe*strafe - 2*strafe*Math.sqrt(2));

                if( Double.isNaN(forward) )
                    forward = forwardSign;
                if( Double.isNaN(strafe) )
                    strafe = strafeSign;

                //Forward speed, Sideways speed, Rotation Speed
                forward = forwardLimiter.calculate(forward) * Robot.swerveDrive.getDrivePace().getValue();
                strafe = strafeLimiter.calculate(strafe) * Robot.swerveDrive.getDrivePace().getValue();

                double rot = -Robot.oi.driver.getRightX(); // need to invert because left is negative, right is positive
                rot = (Math.abs(rot) > Constants.Joystick.DEADBAND) ? rot : 0.0;

                // slow dpwn the rotation by converting the linear response to a curve
                if (rot < 0 ) {
                    rot = rot * -rot;
                } else {
                    rot = rot * rot;
                }

                rot = turnLimiter.calculate(rot) * Robot.swerveConfig.constants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

                // the robot starts facing the driver station so for this year negating y and x
                Robot.swerveDrive.drive(forward*.3, strafe*.3, rot*.3);
            }

    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

}
