// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound <- me when i make up units
                                                                   // for fun
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // the two limelights are named x and y because one is mounted on the
  // x-axis and the other is mounted on the y-axis, two are needed to get an accurate global pose estimate
  public static final String LIMELIGHT_X_IP_ADDRESS = "10.39.84.11"; 
  public static final String LIMELIGHT_Y_IP_ADDRESS = "10.39.84.12";

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  // questnav constants
  public static class QuestNavConstants {
    public static Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );

    public static Transform3d ROBOT_TO_QUEST = new Transform3d(0.0, 6.0, 6.0, new Rotation3d(0.0, 0.0, 0.0));
  }
  // x, y, degrees
  public static double[][] SCORING_POSES = {
    {14.41396, 3.51962, 0.0}, 
    {0.0, 0.0, 0.0}, 
    {0.0, 0.0, 0.0}
  };

  public static final class intake {
    public static final double kP = 0.05;
    public static final double kI = 0.0;
    public static final double kD = 0.01;
    public static final int tolerance = 1;
    public static final int velocityTolerance = 1;
  }

  public static final class intakeArm {
    public static final int rotMotorID = 25;
    public static final double gearRatio = 100.0;

    // PID gains
    public static final double kP = 0.005;
    public static final double kI = 0.0;
    public static final double kD = 0.0001;

    // Feedforward gains
    public static final double kS = 0.0; // Static friction
    public static final double kG = 0.3; // Gravity compensation
    public static final double kV = 0.0; // Velocity feedforward
    public static final double kA = 0.0; // Acceleration feedforward

    // Motion profile constraints
    public static final double maxVelocity = 2.0; // degrees per second
    public static final double maxAcceleration = 1.0; // degrees per second squared

    public static final double tolerance = 0.1; // Position tolerance in degrees
    public static final double velocityTolerance = 1.0; // Velocity tolerance in deg/sec

    public static final Rotation2d retractedPosition = new Rotation2d(Math.toRadians(-4.0));
    public static final Rotation2d deployedPosition = new Rotation2d(Math.toRadians(90.0));
  }

  public static final class shooter {

  }

  public static final class shooterHood {
    public static final double gearRatio = 1.0;

    // PID gains
    public static final double kP = 0.005;
    public static final double kI = 0.0;
    public static final double kD = 0.0001;

    // Feedforward gains
    public static final double kS = 0.0; // Static friction
    public static final double kG = 0.0; // Gravity compensation
    public static final double kV = 0.0; // Velocity feedforward
    public static final double kA = 0.0; // Acceleration feedforward

    // Motion profile constraints
    public static final double maxVelocity = 2.0; // degrees per second
    public static final double maxAcceleration = 1.0; // degrees per second squared

    public static final double tolerance = 0.1; // Position tolerance in degrees
    public static final double velocityTolerance = 1.0; // Velocity tolerance in deg/sec

    public static final double forwardSoftLimit = 35; // degrees
    public static final double reverseSoftLimit = 0;
  }

  public static final class FieldConstants {
    public static final Translation2d kLeftHopper = new Translation2d(
        Units.inchesToMeters(180.11), // X distance from Blue Wall
        Units.inchesToMeters(158.84)  // Y distance from Right side
    );

    public static final Translation2d kRightHopper = new Translation2d(
        Units.inchesToMeters(467.11), 
        Units.inchesToMeters(158.84)
    );
  }
  
}
