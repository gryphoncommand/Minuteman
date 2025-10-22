package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Spindexer.Passthrough;

public class RunPassthrough extends Command {
    private Passthrough passthrough;

    public RunPassthrough(Passthrough passthrough) {
        this.passthrough = passthrough;
        addRequirements(passthrough);
    }
    
    @Override
    public void execute() {
        passthrough.setSpeed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        passthrough.stop();
    }
}
