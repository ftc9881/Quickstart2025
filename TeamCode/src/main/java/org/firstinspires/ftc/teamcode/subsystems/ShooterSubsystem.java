package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx shooter;
    private final CRServo feeder;

    // Constants from your code
    public static double TICKS_PER_REV = 28.0;
    public static double Kf = 0.00017;
    public static double Kp = 0.0005;

    private double targetRPM = 0;
    private boolean isShooting = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        feeder = hardwareMap.get(CRServo.class, "feeder");

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        // --- 1. SHOOTER CONTROL LOOP ---
        if (isShooting && targetRPM > 0) {
            double currentTicksPerSec = shooter.getVelocity();
            double currentRPM = (currentTicksPerSec * 60.0) / TICKS_PER_REV;

            double errorRPM = targetRPM - currentRPM;
            double feedforward = Kf * targetRPM;
            double proportional = Kp * errorRPM;

            double power = feedforward + proportional;

            // Clamp 0 to 1
            power = Math.max(0, Math.min(1, power));
            shooter.setPower(power);
        } else {
            shooter.setPower(0);
        }
    }

    // Call this to turn the shooter ON to a specific speed
    public void enableShooter(double rpm) {
        this.targetRPM = rpm;
        this.isShooting = true;
    }

    // Call this to turn the shooter OFF
    public void stopShooter() {
        this.targetRPM = 0;
        this.isShooting = false;
        shooter.setPower(0);
    }

    // --- 2. FEEDER CONTROL ---

    // Push ring into shooter
    public void feed() {
        feeder.setPower(1.0);
    }

    // Retain ring / Anti-jam (Your -0.2 logic)
    public void stopFeed() {
        feeder.setPower(-0.2);
    }
}
