package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.subsystems.Spindexer.Spindexer;

public class RunSpindexer extends Command {
    private Spindexer spindexer;

    public RunSpindexer(Spindexer spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }
    
    @Override
    public void execute() {
        spindexer.setVelocity(SpindexerConstants.kSpindexerMotorRPM);
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.stop();
    }
}
