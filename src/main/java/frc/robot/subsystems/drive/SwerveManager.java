package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.util.Units;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

import frc.robot.Constants;

public class SwerveManager {
 
    // SwerveDriveKinematics kinematics;

    private final SwerveDrive swerveDrive;

    public SwerveManager(File directory) {

        // phlapjack actually has the same dimensions as the example they give!
        // it seems like these are only used for pathplanner though
        // kinematics = new SwerveDriveKinematics(
        //     new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(12.5)), // Front Left
        //     new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(-12.5)), // Front Right
        //     new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(12.5)), // Back Left
        //     new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(-12.5))  // Back Right
        // );

        // this is yanked straight from the example
        // TODO update this
        boolean blueAlliance = false;
        Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                                                                        Meter.of(4)),
                                                        Rotation2d.fromDegrees(0))
                                        : new Pose2d(new Translation2d(Meter.of(16),
                                                                        Meter.of(4)),
                                                        Rotation2d.fromDegrees(180));

        try {
            swerveDrive = new SwerveParser(directory)
                .createSwerveDrive(Constants.Swerve.MAX_SPEED, startingPose);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public void drive(
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier rotationX) {
        swerveDrive.drive(SwerveMath.scaleTranslation(
            new Translation2d(
                translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                0.8 // 0.8 is currently a magic number TODO figure out where it comes from
            ),
            // according to the example, it's cubed here "for smoother controls"
            Math.pow(rotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
            true, // drive relative to field
            false); // make it closed loop
    }

    public void zeroGyro() {
        AHRS navx = (AHRS) swerveDrive.getGyro().getIMU();
        navx.zeroYaw();
    }
}
