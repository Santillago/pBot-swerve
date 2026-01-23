// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import team.gif.robot.subsystems.drivers.swerve.utilities.SwerveConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double DEBOUNCE_DEFAULT = 0.020;

    //These constants should be referenced via Robot.swerveDrive.getConstants();
    public static final class Mk4Constants extends SwerveConstants {
        @Override
        protected void setConfiguration() {
            MODULE_GEAR_RATIO = 6.75;
            TURNING_MOTOR_GEAR_RATIO = 12.8;
            WHEEL_DIAMETER_INCHES = 4.0;

            FRONT_LEFT_OFFSET = 79.435125;
            FRONT_RIGHT_OFFSET = -20.09625;
            REAR_LEFT_OFFSET = -137.8125;
            REAR_RIGHT_OFFSET = 155.126953125;

            TRACK_LENGTH_INCHES = 24.899;
            TRACK_WIDTH_INCHES = 21.399;

            MASS_KG = 68;
            MOI_KGM2 = 6.883;

            TURN_P = 0.55;
            TURN_FF = 0.01;

            FL_DRIVE_FF = new SimpleMotorFeedforward(0.16095, 2.3837, 0.077757);
            FR_DRIVE_FF = new SimpleMotorFeedforward(0.1645, 2.3928, 0.074191);
            RL_DRIVE_FF = new SimpleMotorFeedforward(0.10265, 2.3955, 0.22997);
            RR_DRIVE_FF = new SimpleMotorFeedforward(0.13952, 2.4217, 0.137);

            AUTO_P_FORWARD = 2.5;
            AUTO_P_ROTATION = 2.5;

            PATHPLANNER_MOTOR_TYPE = DCMotor.getKrakenX60(1);
            PATHPLANNER_CURRENT_LIMIT = 50;
        }
    }

    public static final class Mk3Constants extends SwerveConstants {
        @Override
        protected void setConfiguration() {
            FRONT_LEFT_OFFSET = 42.363;
            REAR_LEFT_OFFSET = 356.309;
            FRONT_RIGHT_OFFSET =  75.234;
            REAR_RIGHT_OFFSET = 19.951;

            TURN_P = 0.4;
            TURN_FF = 0.01;

            FL_P = -0.2;
            FL_FF = 0;

            FL_DRIVE_FF= new SimpleMotorFeedforward(0.16714, 2.7681, 0.41146);
            FR_DRIVE_FF= new SimpleMotorFeedforward(0.10365, 2.7078, 0.49142);
            RL_DRIVE_FF= new SimpleMotorFeedforward(0.10551, 2.8234, 0.48642);
            RR_DRIVE_FF = new SimpleMotorFeedforward(0.073007, 2.75, 0.40028);

            FL_TURN_INVERTED = true;

            MODULE_GEAR_RATIO = 6.68;
            TURNING_MOTOR_GEAR_RATIO = 12.8;
            WHEEL_DIAMETER_INCHES = 3.78;
            DRIVE_ENCODER_CPR = 42; //Neo Motor

            TRACK_LENGTH_INCHES = 22.5;
            TRACK_WIDTH_INCHES = 23;

            PATHPLANNER_MOTOR_TYPE = DCMotor.getNEO(1);
            PATHPLANNER_CURRENT_LIMIT = 40;
        }
    }

    public static final class Joystick {
        public static final double DEADBAND = 0.1;
    }

    public static final class Limelight {
        public static final double TELE_APRILTAG_ALIGNMENT_P = 0.001934;
        public static final double TELE_APRIL_TAG_ALIGNMENT_D = 0.012;
        public static final double TELE_APRILTAG_RANGING_P = 0.0036053377197318524;
        public static double TELE_APRILTAG_DISTANCE_P = 0.1;
        public static double DESIRED_DISTANCE_IN = 55;
        public static final double TELE_LIMELIGHT_ALIGN_BOOST_PERCENT = 1.75;
        //The angle between the horizontal and the center lens
        public static final double LIMELIGHT_MOUNT_ANGLE_DEG = 45;
        //Distance from ground to center of Limelight lens
        public static final double LIMELIGHT_LENS_HEIGHT_INCHES = 21.625;
        //Distance from ground to target
        public static final double LIMELIGHT_GOAL_HEIGHT_INCHES = 25.5;

    }
}