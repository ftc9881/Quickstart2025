package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Runs the intake at a specific power while the command is active.
 * Stops the intake when the command ends.
 */
public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem intake;
    private final double power;

    /**
     * @param intake The subsystem instance
     * @param power Power to run (Positive for In, Negative for Out usually)
     */
    public IntakeCommand(IntakeSubsystem intake, double power) {
        this.intake = intake;
        this.power = power;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setTarget(power);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setTarget(0);
    }

    @Override
    public boolean isFinished() {
        // Run continuously until interrupted (button release)
        return false;
    }
}