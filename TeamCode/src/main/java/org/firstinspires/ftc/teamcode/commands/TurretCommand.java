package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/**
 * A command that moves the turret to a specific setpoint and finishes when there.
 * Use this for binding to buttons (e.g., Button A -> Face Forward).
 */
public class TurretCommand extends CommandBase {

    private final TurretSubsystem turret;
    private final int targetPosition;

    public TurretCommand(TurretSubsystem turret, int targetPosition) {
        this.turret = turret;
        this.targetPosition = targetPosition;
        // Require the turret so this interrupts any manual control
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setTarget(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return turret.isAtTarget();
    }
}