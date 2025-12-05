package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase  {
    private DcMotor intake;
    private double power;

    public IntakeSubsystem (HardwareMap hMap) {
        intake = hMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void periodic(){
        intake.setPower(power);
    }

    public void setTarget(double power) {
        this.power = power;
    }

    public boolean isAtPower() {
        return true;
    }

}
