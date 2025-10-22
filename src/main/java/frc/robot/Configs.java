package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.05, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class SubsystemBaseConfig {
        public static final SparkMaxConfig subsystemConfig = new SparkMaxConfig();

        static {
                subsystemConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(80)
                        .inverted(true)
                        .openLoopRampRate(0)
                        .closedLoopRampRate(0);
                subsystemConfig.encoder
                    .positionConversionFactor(1)
                    .velocityConversionFactor(1);
                subsystemConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.1, 0, 0)
                    .velocityFF(0)
                    .outputRange(-0.5, 0.5);
        }
    }

        public static final class IntakeDeploy {
                public static final SparkFlexConfig IntakeDeployConfig = new SparkFlexConfig();

                static {
                        IntakeDeployConfig
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(80)
                                .inverted(false)
                                .openLoopRampRate(0)
                                .closedLoopRampRate(0);
                        IntakeDeployConfig.encoder
                                .positionConversionFactor(1)
                                .velocityConversionFactor(1);
                        IntakeDeployConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // These are example gains you may need to them for your own robot!
                                .pid(0.05, 0, 0.01)
                                .velocityFF(0)
                                .outputRange(-0.35, 0.35);
                }
        }

        public static final class Shooter {
                public static final SparkFlexConfig ShooterConfig = new SparkFlexConfig();

                static {
                        ShooterConfig
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(80)
                                .inverted(false)
                                .openLoopRampRate(0)
                                .closedLoopRampRate(0);
                        ShooterConfig.encoder
                                .positionConversionFactor(1)
                                .velocityConversionFactor(1);
                        ShooterConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // These are example gains you may need to them for your own robot!
                                .pid(0.5, 0, 0.01)
                                .velocityFF(0)
                                .outputRange(-0.99, 0.99);
                }
        }

        public static final class Spindexer {
                public static final SparkFlexConfig SpindexerConfig = new SparkFlexConfig();

                static {
                        SpindexerConfig
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(80)
                                .inverted(false)
                                .openLoopRampRate(0)
                                .closedLoopRampRate(0);
                        SpindexerConfig.encoder
                                .positionConversionFactor(1)
                                .velocityConversionFactor(1);
                        SpindexerConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                // These are example gains you may need to them for your own robot!
                                .pid(0.5, 0, 0.01)
                                .velocityFF(0)
                                .outputRange(-0.9, 0.9);
                }
        }
}
