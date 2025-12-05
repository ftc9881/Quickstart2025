package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;

/**
 * Runs the feeder servo at a specific power while the command is active.
 * Stops the feeder when the command ends.
 */
public class FeederCommand extends CommandBase {

    private final FeederSubsystem feeder;
    private final double power;

    public FeederCommand(FeederSubsystem feeder, double power) {
        this.feeder = feeder;
        this.power = power;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feeder.setTarget(power);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setTarget(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}