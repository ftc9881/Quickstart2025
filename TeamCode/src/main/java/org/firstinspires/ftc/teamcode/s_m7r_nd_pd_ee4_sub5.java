package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="M7: s+ m7r nd pd ee4 sub5", group="Robot")
//@Disabled
public class s_m7r_nd_pd_ee4_sub5 extends LinearOpMode {
    /* Copyright (c) 2017 FIRST. All rights reserved.
     *
     * Redistribution and use in source and binary forms, with or without modification,
     * are permitted (subject to the limitations in the disclaimer below) provided that
     * the following conditions are met:
     *
     * Redistributions of source code must retain the above copyright notice, this list
     * of conditions and the following disclaimer.
     *
     * Redistributions in binary form must reproduce the above copyright notice, this
     * list of conditions and the following disclaimer in the documentation and/or
     * other materials provided with the distribution.
     *
     * Neither the name of FIRST nor the names of its contributors may be used to endorse or
     * promote products derived from this software without specific prior written permission.
     *
     * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
     * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
     * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
     * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
     * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
     * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
     * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
     */

//    public ColorSensor sensor = null;

    /* Declare OpMode members. */
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor rightRear = null;
    public DcMotor leftRear = null;
    public DcMotorEx shooter1 = null;

    public DcMotor intake = null;
    public DcMotor turret = null;
    public CRServo feeder = null;

    public static double TICKS_PER_REV = 28.0;
    public static double TARGET_RPM = 5000.0;

    // Feedforward term (helps get close to target quickly)
    public static double Kf = 0.00017;
    public static double Kp = 0.0005;

    boolean isShooting = false;
    boolean lastA = false;
    boolean lastB = false;
    boolean close = true;

    private ElapsedTime timer = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // Define and Initialize Motors


        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");

        intake = hardwareMap.get(DcMotor.class, "intake");
        turret = hardwareMap.get(DcMotor.class, "turret");
        feeder = hardwareMap.get(CRServo.class, "feeder");


        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //        sensor = hardwareMap.get(ColorSensor.class, "Color");
//            DigitalChannel pin0 = hardwareMap.digitalChannel.get("digital0");
//            DigitalChannel pin1 = hardwareMap.digitalChannel.get("digital1");

        // Drive

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter1.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.REVERSE);

        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        int turretPos = 0;
        turret.setTargetPosition(turretPos);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(.5);


//            timer.reset();

        // Intake
        double intakePower = 0;

        waitForStart();

        double power = 0;

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.08; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * 1.0; // turn speed reduction


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.left_bumper) {
                frontLeftPower *= .41;
                backLeftPower *= .41;
                frontRightPower *= .41;
                backRightPower *= .41;
            }

            if (gamepad1.a && !lastA) {
                isShooting = !isShooting;
                close = true;
                TARGET_RPM = 3850;
            }
            lastA = gamepad1.a;

            if (gamepad1.b && !lastB) {
                isShooting = !isShooting;
                close = false;
                TARGET_RPM = 4967;
            }
            lastA = gamepad1.a;

            double currentTicksPerSec = shooter1.getVelocity();

            double currentRPM = (currentTicksPerSec * 60.0) / TICKS_PER_REV;

            if (isShooting) {
                // 1. Calculate the error (Difference between what we want and what we have)
                // Note: We calculate error in RPM to keep Kp numbers intuitive
                double errorRPM = TARGET_RPM - currentRPM;

                // 2. Calculate Feedforward
                // This is the "base" power to keep the wheel spinning at target speed
                double feedforward = Kf * TARGET_RPM;

                // 3. Calculate Proportional
                // This adds extra power if we are too slow, or reduces if too fast
                double proportional = Kp * errorRPM;

                // 4. Sum them up
                power = feedforward + proportional;

                // 5. Clamp the power between 0 and 1 (flywheels usually don't need reverse here)
                power = Math.max(0, Math.min(1, power));
            } else {
                power = 0;
            }

            if (gamepad1.right_trigger > .1) {
                intakePower = gamepad1.right_trigger;
            } else if (gamepad1.left_trigger > .1) {
                intakePower = gamepad1.left_trigger;
            } else {
                intakePower = 0;
            }
            if(gamepad1.right_bumper) {
                feeder.setPower(1);
                intakePower = .67;
            } else {
                feeder.setPower(-.2);
            }


            if(gamepad1.dpad_up) {
                turretPos = 0;
            }
            if(gamepad1.dpad_right) {
                turretPos = -250;
            }
            if(gamepad1.dpad_left) {
                turretPos = 250;
            }

            turret.setTargetPosition(turretPos);

            shooter1.setPower(power);

            intake.setPower(intakePower);

            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            telemetry.addData("actual velocity", currentRPM);
            telemetry.addData("target velocity", TARGET_RPM);

            telemetry.update();
        }
    }
}

