package team.gif.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team.gif.robot.commands.drivetrain.DriveToRange;

public class UI {
    /**
     *  Widgets (e.g. gyro, text, True/False flags),
     *  buttons (e.g. SmartDashboard.putData("Reset", new ResetHeading()); ),
     *  and Chooser options (e.g. auto mode, auto delay)
     *
     *  Placed in SmartDashboard network table
     *  After dashboard loads for the first time, manually move items from network table onto respective dashboard tab
     *  and save file as "YYYY elastic-layout.json"
     */
    public UI() {
        SmartDashboard.putNumber("Limelight/Dist_P", Constants.Limelight.TELE_APRILTAG_DISTANCE_P);
        SmartDashboard.putNumber("Limelight/Desired", Constants.Limelight.TELE_APRILTAG_DESIRED_DISTANCE_INCHES);

    }

    /**
     * Widgets which are updated periodically should be placed here
     *
     * Convenient way to format a number is to use putString w/ format:
     *     SmartDashboard.putString("Elevator", String.format("%11.2f", Elevator.getPosition());
     */
    public void update() {
        SmartDashboard.putNumber("Limelight/X Offset", Robot.limelight.getXOffset());
        SmartDashboard.putNumber("Limelight/Y Offset", Robot.limelight.getYOffset());
        SmartDashboard.getNumber("Limelight/Dist_P", Constants.Limelight.TELE_APRILTAG_DISTANCE_P);
        SmartDashboard.getNumber("Limelight/Desired_Dist", Constants.Limelight.TELE_APRILTAG_DESIRED_DISTANCE_INCHES);
        SmartDashboard.putBoolean("Swerve/Align Mode", Robot.swerveDrive.alignMode);
    }
}
