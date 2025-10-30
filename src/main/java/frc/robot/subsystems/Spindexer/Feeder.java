package frc.robot.subsystems.Spindexer;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;

public class Feeder extends SubsystemBase {
    SparkFlex feederMotor = new SparkFlex(SpindexerConstants.kFeederMotorID, MotorType.kBrushless);

    
}
