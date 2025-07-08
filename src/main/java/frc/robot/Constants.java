// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int ARM_MOTOR_ID = 0; // TODO: find these ids
    public static final int EFFECTOR_MOTOR_ID = 0;
    public static final double MAX_ARM_VELOCIY = 0.0; // measured in rpm TODO
    public static final double MAX_ARM_ACCELERATION = 0.0; // measured in rpm per second TODO
    public static final double ALLOWED_CLOSED_LOOP_ERROR = 0.0; // measured in rotations TODO
  }

  public static class CoralArmSetpoints {
    public static final double STOW = 0.0; // TODO figure these out
    public static final double SCORE = 0.0;
    public static final double INTAKE = 0.0;
  }

  public static class CoralArmPID {
    public static final double P = 0.0; // TODO figure these out
    public static final double I = 0.0;
    public static final double D = 0.0;
  }

  public static class AlgaeHardware {
    public static final int INTAKE_MOTOR_ID = 0; // TODO: find these ids
    public static final int ARM_MOTOR_ID = 0;
    public static final int SHOOTER_MOTOR_ID = 0;
    public static final int BEAM_BREAK_ID = 0;
    public static final double MAX_ARM_VELOCIY = 0.0; // measured in rpm TODO
    public static final double MAX_ARM_ACCELERATION = 0.0; // measured in rpm per second TODO
    public static final double ALLOWED_CLOSED_LOOP_ERROR = 0.0; // measured in rotations TODO
  }

  public static class AlgaeArmSetpoints {
    public static final double STOW = 0.0; // TODO figure these out
    public static final double DESCORE = 0.0;
    public static final double INTAKE = 0.0;
  }

  public static class AlgaeArmPID {
    public static final double P = 0.0; // TODO figure these out
    public static final double I = 0.0;
    public static final double D = 0.0;
  }

  public static class ClimbHardware {
    public static final int ARM_MOTOR_ID = 0; // TODO: find these ids
    public static final double MAX_ARM_VELOCIY = 0.0; // measured in rpm TODO
    public static final double MAX_ARM_ACCELERATION = 0.0; // measured in rpm per second TODO
    public static final double ALLOWED_CLOSED_LOOP_ERROR = 0.0; // measured in rotations TODO
  }

  public static class ClimbMotorSetpoints {
    public static final double EXTEND = 0.0; // TODO figure these out
    public static final double RETRACT = 0.0;
  }

  public static class ClimbMotorPID {
    public static final double P = 0.0; // TODO figure these out
    public static final double I = 0.0;
    public static final double D = 0.0;
  }

  public static class Swerve {
    public static final double MAX_SPEED = 0.0; // TODO make this not zero
  }
}
