package team.gif.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import team.gif.lib.drivePace;
import team.gif.robot.Constants;
import team.gif.robot.Robot;

public class TagAlignment extends Command {

    private PIDController alignPID, rangePID;

    public TagAlignment() {
        super();
        this.alignPID = new PIDController(Constants.Limelight.TELE_APRILTAG_ALIGNMENT_P, 0, 0);
        this.rangePID = new PIDController(Constants.Limelight.TELE_APRILTAG_RANGING_P, 0, 0);
        addRequirements(Robot.swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.swerveDrive.setDrivePace(drivePace.COAST_RR);
    }

    // Called every time the scheduler runs (~20ms) while the command is scheduled
    @Override
    public void execute() {
        //Get x offset from the apriltag and use P control to reach 0 offset;
        double targetAngularVelocity = alignPID.calculate(Robot.limelight.getXOffset(), 0);
        //Convert to radians per second
        targetAngularVelocity *= Robot.swerveConfig.constants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

        //NOTE: Driving to the target using this method is very finicky and should really not be used, look at the DriveToRange command for a better example
        //Get Y offset (from target to crosshair) and use P control to reach 0 offset
        //Keep in mind, that because the target and the limelight are mounted at different heights, using y offset to range is a viable choice. If they were mounted at similar heights, then you would need to use area. //-0.71
        double targetLinearVelocity = rangePID.calculate(Robot.limelight.getYOffset(), 0);
        //Convert to meters per second
        targetLinearVelocity *= Robot.swerveConfig.constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        //Invert values since positive values go backward
        targetLinearVelocity *= 1.0;

        //Apply boosts since the original values are too slow
        targetLinearVelocity *= Constants.Limelight.TELE_LIMELIGHT_ALIGN_BOOST_PERCENT;
        targetAngularVelocity *= Constants.Limelight.TELE_LIMELIGHT_ALIGN_BOOST_PERCENT;


        //targetingAngularVelocity will eventually become 0
        Robot.swerveDrive.drive(targetLinearVelocity, 0, targetAngularVelocity);
    }

    // Return true when the command should end, false if it should continue. Runs every ~20ms.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.swerveDrive.setDrivePace(drivePace.COAST_FR);
    }
}
