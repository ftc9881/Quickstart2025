package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FeederSubsystem extends SubsystemBase  {
    private CRServo feeder;
    private double power;

    public FeederSubsystem (HardwareMap hMap) {
        feeder = hMap.get(CRServo.class, "feeder");

    }

    @Override
    public void periodic(){
        feeder.setPower(power);
    }

    public void setTarget(double power) {
        this.power = power;
    }

    public boolean isAtPower() {
        return true;
    }

}
