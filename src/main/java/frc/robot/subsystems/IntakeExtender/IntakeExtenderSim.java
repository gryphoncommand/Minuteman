package frc.robot.subsystems.IntakeExtender;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;



// This code is heavily based on the WPILib SingleJointedArmSim example, as well as the ArmSim example from the KrakenX60 motor documentation
// It also takes heavy inspiration from team 1114's TalonFX simulation code
public class IntakeExtenderSim extends SubsystemBase implements IntakeExtenderIO {
    private enum ControlMode { DutyCycle, Velocity, Position, Voltage }

    private final ProfiledPIDController pivotController =
            new ProfiledPIDController(4.0, 0.0, 0.1, new TrapezoidProfile.Constraints(4.0, 4.0));

    private final DCMotor pivotGearbox = DCMotor.getKrakenX60(1); // gearbox object (no reduction applied here)

    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.kPivotMotorID);

    private final Encoder pivotEncoder = new Encoder(1, 2);
    private final EncoderSim pivotEncoderSim = new EncoderSim(pivotEncoder);

    private double targetReference = 0.0; // units: rotations when used externally (see methods)
    private ControlMode currentControlMode = ControlMode.DutyCycle;

    private final double kGearRatio = IntakeConstants.kGearRatio; // motor rotations per arm rotation (e.g. 15)
    private final double jKgM2 = SingleJointedArmSim.estimateMOI(
            Units.inchesToMeters(IntakeConstants.kArmLengthInches),
            Units.lbsToKilograms(15.0)); // mass guess - tune as needed

    private final SingleJointedArmSim wristSim = new SingleJointedArmSim(
            pivotGearbox,                   // motor model
            kGearRatio,                     // gearbox reduction (motor:arm)
            jKgM2,                          // moment of inertia (kg*m^2)
            Units.inchesToMeters(IntakeConstants.kArmLengthInches),
            -Math.PI / 2.0,                 // min angle (rad), adjust if needed
            Math.PI * 1.5,                  // max angle (rad)
            true,                           // add gravity
            0.0                             // measurement noise (rad)
    );

    private final Mechanism2d wristSimMechanism = new Mechanism2d(Units.inchesToMeters(30), Units.inchesToMeters(30));
    private final MechanismRoot2d wristHome = wristSimMechanism.getRoot("Base", Units.inchesToMeters(15), Units.inchesToMeters(15));
    public final MechanismLigament2d wristArm = wristHome.append(
            new MechanismLigament2d("Intake", Units.inchesToMeters(IntakeConstants.kArmLengthInches), 0.0));

    public IntakeExtenderSim() {
        pivotController.reset(0.0);

        SmartDashboard.putData("Intake Mech2D", wristSimMechanism);
        SmartDashboard.putNumber("Intake Stow Angle (rot)", IntakeConstants.kStowedAngle);
        SmartDashboard.putNumber("Intake Extension Angle (rot)", IntakeConstants.kIntakeAngle);

        // Encoder: WPILib Encoder.setDistancePerPulse should be in rotations per pulse if you want getDistance() to return rotations.
        // The WPILib Encoder "pulse" is a full cycle (4 edges) so setDistancePerPulse = 1 / (CPR * gearRatioIfEncoderOnMotor)
        // Here we assume encoder CPR is in IntakeConstants.kEncoderCPR and encoder is on the motor shaft.
        double cpr = IntakeConstants.kEncoderCPR;
        double rotationsPerPulse = 1.0 / cpr;
        double armRotationsPerPulse = rotationsPerPulse / kGearRatio;
        pivotEncoder.setDistancePerPulse(armRotationsPerPulse);

        setPosition(IntakeConstants.kStowedAngle);

        currentControlMode = ControlMode.DutyCycle;
    }

    @Override
    public void periodic() {
        double motorPercent = pivotMotor.get();
        double motorVoltage = motorPercent * RobotController.getBatteryVoltage();

        wristSim.setInput(motorVoltage);
        wristSim.update(0.02); // 20 ms periodic

        double armAngleRad = wristSim.getAngleRads();
        double armRotations = Units.radiansToRotations(armAngleRad);
        pivotEncoderSim.setDistance(armRotations);

        double displayDeg = Math.toDegrees(armAngleRad);
        wristArm.setAngle(displayDeg);

        if (currentControlMode == ControlMode.Position && DriverStation.isEnabled()) {
            double targetRad = Units.rotationsToRadians(targetReference);
            double pidOutput = pivotController.calculate(armAngleRad, targetRad);
            pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
            pivotMotor.set(pidOutput);
        } else if (currentControlMode == ControlMode.Velocity && DriverStation.isEnabled()) {
            double targetRPM = targetReference;
            double currentRadPerSec = wristSim.getVelocityRadPerSec();
            double targetRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(targetRPM);
            double velError = targetRadPerSec - currentRadPerSec;
            double kPvel = 0.002; // tune
            double out = kPvel * velError;
            out = Math.max(-1.0, Math.min(1.0, out));
            pivotMotor.set(out);
        }

        SmartDashboard.putNumber("Intake Position (rot)", getPosition());
        SmartDashboard.putNumber("Intake Speed (RPM)", getVelocity());

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(wristSim.getCurrentDrawAmps()));
    }


    @Override
    public void set(double speed) {
        pivotMotor.set(speed);
        currentControlMode = ControlMode.DutyCycle;
    }

    @Override
    public void setVelocity(double velocityRPM) {
        this.targetReference = velocityRPM;
        this.currentControlMode = ControlMode.Velocity;
    }

    @Override
    public void setPosition(double positionRotations) {
        SmartDashboard.putNumber("Requested Wrist Position (rot)", positionRotations);
        this.targetReference = positionRotations;
        this.currentControlMode = ControlMode.Position;

        double goalRad = Units.rotationsToRadians(positionRotations);
        pivotController.reset(Units.radiansToRotations(wristSim.getAngleRads())); // reset with current state if desired
        pivotController.setGoal(goalRad);
    }

    @Override
    public void setVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
        currentControlMode = ControlMode.Voltage;
    }

    @Override
    public void setEncoderPosition(double positionRotations) {
        double angleRad = Units.rotationsToRadians(positionRotations);
        wristSim.setState(angleRad, 0.0);
        pivotEncoderSim.setDistance(positionRotations);
    }

    @Override
    public double getVelocity() {
        double radPerSec = wristSim.getVelocityRadPerSec();
        return Units.radiansPerSecondToRotationsPerMinute(radPerSec);
    }

    @Override
    public double getPosition() {
        return pivotEncoder.getDistance();
    }

    @Override
    public boolean atTarget(double threshold) {
        if (currentControlMode == ControlMode.Velocity) {
            return Math.abs(getVelocity() - targetReference) < threshold;
        } else if (currentControlMode == ControlMode.Position) {
            return Math.abs(getPosition() - targetReference) < threshold;
        } else {
            return false;
        }
    }

    @Override
    public SubsystemBase returnSubsystem() {
        return this;
    }

    @Override
    public Optional<MechanismLigament2d> returnLigament() {
        return Optional.of(wristArm);
    }
}
