package org.firstinspires.ftc.teamcode.teleop.mechTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

@Disabled
@Config
@TeleOp(group = "mech test")
public class ShooterTest extends OpMode {
    Robot robot;
    GamepadMapping controls;

    public static boolean backShoot = false;
    public static double power = 0;

    private Telemetry dashboardTelemetry;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        controls.update();

//        if (backShoot) {
//            if(controls.shootBack.value()) {
//                robot.shooter.shootFromBack();
//            } else {
//                robot.transfer.off();
//            }
//        }
//
//        if (!backShoot) {
//            if(controls.shootTriangle.value()) {
//                robot.shooter.shootFromFront();
//            } else {
//                robot.transfer.off();
//            }
//        }


//        robot.shooter.setShooterPower(-1);
//
//        if (robot.shooter.outtake.getVelocity() <= 1500) {
//            robot.transfer.transferOn();
//        }

//        if (controls.shootBack.value()) {
//            robot.shooter.shootFromFront();
//        }

        //robot.shooter.setShooterPower(power);
        robot.shooter.outtake1.setVelocity(500);


        dashboardTelemetry.addData("backShootValue", Shooter.outtakeVels.HARDCODED_SHOOT_BACK.getOuttakeVel());
        // once we get to 1500 velo
        dashboardTelemetry.addData("velo", robot.shooter.outtake1.getVelocity());
        dashboardTelemetry.addData("Encoder:",robot.shooter.outtake1.getCurrentPosition());
        dashboardTelemetry.update();
    }
}
