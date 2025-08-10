// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double EPSILON = 0.000001; // lgtm

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CoralArmHardware {
    public static final int ARM_MOTOR_ID = 52;
    public static final int EFFECTOR_MOTOR_ID = 51;
    public static final double ALLOWED_CLOSED_LOOP_ERROR = 0.05;
    public static final Current ARM_MOTOR_CURRENT_LIMIT = Units.Amps.of(40);
    public static final Current ROLLER_MOTOR_CURRENT_LIMIT = Units.Amps.of(40);
    public static final Current ARM_STALL_CURRENT = Units.Amps.of(20);
    public static final Current ROLLER_STALL_CURRENT = Units.Amps.of(30);
  }

  public static class CoralArmSetpoints {
    public static final double STOW = -0.5;
    public static final double SCORE = -5.0;
    public static final double INTAKE = -17.35;
    public static final double INTAKE_HIGH = -2.5;
  }

  public static class CoralArmPID {
    public static final double P = 0.08;
    public static final double I = 0.0;
    public static final double D = 0.0;
  }

  public static class CoralArmConfig {
    public static final double ROLLER_DEADBAND_TIME = 0.5;
    public static final double ARM_MOTOR_DEADBAND_TIME = 0.5;
    public static final double SCORE_EJECT_TIME = 0.25;
  }

  public static class ClimbHardware {
    public static final int CLIMB_MOTOR_ID = 61;
    public static final Current CLIMB_MOTOR_CURRENT_LIMIT = Units.Amps.of(70);
    public static final double DEADBAND_TIME = 0.25;
    public static final double END_TIMEOUT = 2;
  }
  
  public static class Swerve {
    public static final double MAX_SPEED = 4.4196; // 14.5 feet to meters
    public static final double DEADBAND = 0.08;
    public static final double TRANSLATION_SCALE = 0.8;
    public static final double GIMP_SCALE = 0.45;
    public static final double AUTO_DRIVE_TIME = 2.5;
    public static final double AUTO_FORWARD_SPEED = -0.1;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = edu.wpi.first.math.util.Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = edu.wpi.first.math.util.Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 21;
    public static final int kRearLeftDrivingCanId = 41;
    public static final int kFrontRightDrivingCanId = 11;
    public static final int kRearRightDrivingCanId = 31;

    public static final int kFrontLeftTurningCanId = 22;
    public static final int kRearLeftTurningCanId = 42;
    public static final int kFrontRightTurningCanId = 12;
    public static final int kRearRightTurningCanId = 32;

    public static final boolean kGyroReversed = true;
  }
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
