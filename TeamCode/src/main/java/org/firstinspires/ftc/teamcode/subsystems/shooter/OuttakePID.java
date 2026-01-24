package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;

@TeleOp
public class OuttakePID extends LinearOpMode {

    DcMotorEx outtake;
    Robot robot;
    GamepadMapping gamepad;

    @Override
    public void runOpMode() {
        gamepad = new GamepadMapping(gamepad1, gamepad2);
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        robot = new Robot(hardwareMap, gamepad);
        
        // Set PIDF (start with defaults, tune later)
        outtake.setVelocityPIDFCoefficients(0.2, 2.0, 0.002, 0.0);

        waitForStart();

        double time = 0;
        while (opModeIsActive()) {
            time += 0.02; // ~50 Hz loop

            //sine wave/variable setpoint between 2000 and 5000 ticks/sec
            double targetVel = 3500 + 1500 * Math.sin(2 * Math.PI * 0.5 * time);

            // Send target to REV Hub PID
            outtake.setVelocity(targetVel);

            // Read actual velocity
            double actualVel = outtake.getVelocity();

            // Telemetry
            telemetry.addData("Target (ticks/s): ", targetVel);
            telemetry.addData("Actual (ticks/s): ", actualVel);
            telemetry.update();

            sleep(20);
        }
    }
}