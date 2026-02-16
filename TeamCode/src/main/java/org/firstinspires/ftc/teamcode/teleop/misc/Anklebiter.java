package org.firstinspires.ftc.teamcode.teleop.misc;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled
@TeleOp
public class Anklebiter extends OpMode {

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        leftFront.setPower((drive + strafe + turn) * 0.5);
        leftBack.setPower((drive - strafe + turn) * 0.5);
        rightFront.setPower((drive - strafe - turn) * 0.5);
        rightBack.setPower((drive + strafe - turn) * 0.5);
    }
}
