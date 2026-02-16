package org.firstinspires.ftc.teamcode.teleop.mechTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;

@Disabled
@Config
@TeleOp(group = "mech tests")
public class  TransferTest extends OpMode {
    Robot robot;
    GamepadMapping controls;
    public static double power = 0;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
    }

    @Override
    public void loop() {
        controls.update();

        robot.shooter.setShooterVelocity(-1100);
        robot.intake.intakeOn();

        robot.transfer.frontTransfer.setVelocity(power);
        robot.transfer.backTransfer.setPower(1);

//i may be wrong but idk if calling rr action like on and off willl work here
//im prolly wrong (dis ishaan)
//        if(controls.failSafeReset.value()) {
//            robot.transfer.transferOn();
//        } else {
//            robot.transfer.transferOff();
//        }

    }
}
