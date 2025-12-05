package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretSubsystem extends SubsystemBase  {
    private DcMotor turret;
    private int target;
    private int tolerance = 5;

    public TurretSubsystem (HardwareMap hMap) {
        turret = hMap.get(DcMotor.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.67);
    }

    @Override
    public void periodic(){
        turret.setTargetPosition(target);
    }

    public void setTarget(int target) {
        this.target = target;
    }

    public int getPos() {
        return turret.getCurrentPosition();
    }

    public boolean isAtTarget() {
        return (turret.getCurrentPosition() + tolerance > target && turret.getCurrentPosition() - tolerance < target);
    }

}
