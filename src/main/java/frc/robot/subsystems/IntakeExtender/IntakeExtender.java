package frc.robot.subsystems.IntakeExtender;

import java.util.Optional;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeExtender extends SubsystemBase implements IntakeExtenderIO {
    private final SparkFlex extenderMotor = new SparkFlex(IntakeConstants.kPivotMotorID, MotorType.kBrushless);
    private final RelativeEncoder extenderEncoder = extenderMotor.getEncoder();
    private final SparkClosedLoopController pid;

    private double targetReference;
    private ControlType currentControlType;

    private final Mechanism2d extenderMech2d = new Mechanism2d(Units.inchesToMeters(40), Units.inchesToMeters(30));
    private final MechanismRoot2d extenderRoot = extenderMech2d.getRoot("Base", Units.inchesToMeters(15), Units.inchesToMeters(15));
    private final MechanismLigament2d extenderArm = extenderRoot.append(
        new MechanismLigament2d("Intake Arm", Units.inchesToMeters(12), 0)
    );

    public IntakeExtender() {
        SmartDashboard.putNumber("Intake Stow Angle", IntakeConstants.kStowedAngle);
        SmartDashboard.putNumber("Intake Extension Angle", IntakeConstants.kIntakeAngle);

        extenderMotor.configure(Configs.IntakeDeploy.IntakeDeployConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        pid = extenderMotor.getClosedLoopController();

        targetReference = 0;
        currentControlType = ControlType.kDutyCycle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeExtender Position", extenderEncoder.getPosition());
        SmartDashboard.putNumber("IntakeExtender Velocity", extenderEncoder.getVelocity());
        SmartDashboard.putString("IntakeExtender State", "Real");

        extenderArm.setAngle(-Units.rotationsToDegrees(extenderEncoder.getPosition() / IntakeConstants.kGearRatio));
    }

    @Override
    public void set(double speed) {
        extenderMotor.set(speed);
        currentControlType = ControlType.kDutyCycle;
    }

    @Override
    public void setVelocity(double velocity) {
        pid.setReference(velocity, ControlType.kVelocity);
        targetReference = velocity;
        currentControlType = ControlType.kVelocity;
    }

    @Override
    public void setPosition(double position) {
        pid.setReference(position, ControlType.kPosition);
        SmartDashboard.putNumber("Requested Intake Position", position);
        targetReference = position;
        currentControlType = ControlType.kPosition;
    }

    @Override
    public void setVoltage(double voltage) {
        extenderMotor.setVoltage(voltage);
        currentControlType = ControlType.kVoltage;
    }

    @Override
    public void setEncoderPosition(double position) {
        extenderEncoder.setPosition(position);
    }

    @Override
    public double getVelocity() {
        return extenderEncoder.getVelocity();
    }

    @Override
    public double getPosition() {
        return extenderEncoder.getPosition();
    }

    @Override
    public boolean atTarget(double threshold) {
        if (currentControlType == ControlType.kVelocity) {
            return Math.abs(getVelocity() - targetReference) < threshold;
        } else if (currentControlType == ControlType.kPosition) {
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
        return Optional.of(extenderArm);
    }
}
