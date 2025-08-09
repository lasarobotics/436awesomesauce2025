package frc.robot.subsystems.drive;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import org.lasarobotics.fsm.StateMachine;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class DriveSubsystem extends StateMachine implements AutoCloseable {

    private final MAXSwerve swerveDrive;

    public enum DriveSubsystemStates implements SystemState {
        NOTHING {
            @Override
            public SystemState nextState() {
                return this;
            }
        },
        AUTO {
            Timer m_autoTimer = new Timer();

            @Override
            public void initialize() {
                m_autoTimer.reset();
                m_autoTimer.start();
            }

            @Override
            public void execute() {
                if (!m_autoTimer.hasElapsed(Constants.Swerve.AUTO_DRIVE_TIME)) {
                    // left stick backwards 0.25
                    // (towards the driver station)
                    // horizontally centered
                    // no rotation
                    getInstance().drive(0, Constants.Swerve.AUTO_FORWARD_SPEED, 0);
                } else {
                    getInstance().drive(0, 0, 0);
                }
            }

            @Override
            public SystemState nextState() {
                if (DriverStation.isAutonomous()) return this;

                return TELEOP;
            }
        },
        TELEOP {
            @Override
            public void execute() {
                getInstance().drive();
            }

            @Override
            public SystemState nextState() {
                return this;
            }
        }
    }

    private static DriveSubsystem s_driveSubsystemInstance;
    private DoubleSupplier m_leftX;
    private DoubleSupplier m_leftY;
    private DoubleSupplier m_rightX;
    private BooleanSupplier m_reset;

    public static DriveSubsystem getInstance() {
        if (s_driveSubsystemInstance == null) {
            s_driveSubsystemInstance = new DriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
        }
        return s_driveSubsystemInstance;
    }

    public DriveSubsystem(
            File directory) {
        super(DriveSubsystemStates.AUTO);
        swerveDrive = new MAXSwerve();
    }

    public void configureBindings(
            DoubleSupplier leftX,
            DoubleSupplier leftY,
            DoubleSupplier rightX,
            BooleanSupplier reset) {
        m_leftX = leftX;
        m_leftY = leftY;
        m_rightX = rightX;
        m_reset = reset;
    }

    public void drive() {
        Logger.recordOutput("DriveSubsystem/Inputs/LeftX", m_leftX.getAsDouble());
        Logger.recordOutput("DriveSubsystem/Inputs/LeftY", m_leftY.getAsDouble());
        Logger.recordOutput("DriveSubsystem/Inputs/RightX", m_rightX.getAsDouble());
        
        swerveDrive.drive(
            m_leftX.getAsDouble() * Constants.Swerve.GIMP_SCALE * Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.Swerve.TRANSLATION_SCALE,
            m_leftY.getAsDouble() * Constants.Swerve.GIMP_SCALE * Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.Swerve.TRANSLATION_SCALE,
            m_rightX.getAsDouble() * Constants.Swerve.GIMP_SCALE * Constants.DriveConstants.kMaxAngularSpeed,
            true
        );
    }

    public void drive(double leftX, double leftY, double rightX) {
        Logger.recordOutput("DriveSubsystem/DriveManual/LeftX", m_leftX.getAsDouble());
        Logger.recordOutput("DriveSubsystem/DriveManual/LeftY", m_leftY.getAsDouble());
        Logger.recordOutput("DriveSubsystem/DriveManual/RightX", m_rightX.getAsDouble());
        
        swerveDrive.drive(
            leftX * Constants.Swerve.GIMP_SCALE * Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.Swerve.TRANSLATION_SCALE,
            leftY * Constants.Swerve.GIMP_SCALE * Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.Swerve.TRANSLATION_SCALE,
            rightX * Constants.Swerve.GIMP_SCALE * Constants.DriveConstants.kMaxAngularSpeed,
            true
        );
    } 

    public void periodic() {
        if (m_reset.getAsBoolean())
            zeroGyro();
        swerveDrive.periodic();
        Logger.recordOutput("DriveSubsystem/pose", swerveDrive.getPose());
        Logger.recordOutput("DriveSubsystem/heading", swerveDrive.getHeading());
        Logger.recordOutput(getName() + "/state", getState().toString());
    }

    public void zeroGyro() {
        swerveDrive.zeroHeading();
    }

    public void offset180() {
        swerveDrive.offset180();
    }

    public void close() {}
}

//henry wuz here