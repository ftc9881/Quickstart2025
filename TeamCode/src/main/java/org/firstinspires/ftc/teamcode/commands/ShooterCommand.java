package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final double targetRPM;

    public ShooterCommand(ShooterSubsystem subsystem, double targetRPM) {
        this.shooterSubsystem = subsystem;
        this.targetRPM = targetRPM;
        // Require the subsystem so no other command can use the shooter at the same time
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.enableShooter(targetRPM);
    }

    @Override
    public void execute() {
        // The PIDF loop is handled automatically in the subsystem's periodic()
        // We don't need to put anything here!
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted (or button released/toggled)
    }
}