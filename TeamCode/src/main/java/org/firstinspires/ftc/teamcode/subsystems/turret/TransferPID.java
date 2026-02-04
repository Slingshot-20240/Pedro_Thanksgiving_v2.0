package org.firstinspires.ftc.teamcode.subsystems.turret;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.fsm.FSM;


@Config
@TeleOp(name = "TransferPID", group = "Testing")
public class TransferPID extends OpMode {
//test
    DcMotorEx frontTransfer;
    //public static double p1 = 600, i1 = 0.0, d1 = 0.0, f1 = 40;
    public static double p1 = 0, i1 = 0, d1 = 0, f1 = 0;
    public static int targetVel = 100;
    private Telemetry dashboardTelemetry;

    Robot robot;
    GamepadMapping controls;
    FSM fsm;
    Drivetrain drivetrain;

    @Override
    public void init() {
        double time = 0;
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        frontTransfer = hardwareMap.get(DcMotorEx.class, "transferF");
 
        // Set PIDF (start with defaults, tune later)
        frontTransfer.setVelocityPIDFCoefficients(578, 0, 0, 70);
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);

        frontTransfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontTransfer.setDirection(DcMotorSimple.Direction.FORWARD);

//        fsm = new FSM(hardwareMap, controls, robot);
//
//        drivetrain = new Drivetrain(hardwareMap, robot.imu, controls);
    }

    @Override
    public void loop() {
        //time += 0.02; // ~50 Hz loop

        // Updates driver controls here as well
//        drivetrain.update();
//        // Updates all other controls
//        controls.update();

        //robot.intake.intakeOn();

//        if (controls.transfer.locked()) {
//            robot.transfer.transferOn();
//        } else {
//            robot.transfer.hotDog();
//        }
//
//        if (controls.outtake.locked()) {
//            robot.intake.intakeReverse();
//        } else {
//            robot.intake.intakeOn();
//        }

        //sine wave/variable setpoint between 2000 and 5000 ticks/sec
        //double targetVel = 3500 + 1500 * Math.sin(2 * Math.PI * 0.5 * time);

      // Send target to REV Hub PID
        frontTransfer.setVelocity(targetVel);
        frontTransfer.setVelocityPIDFCoefficients(p1, i1, d1, f1);


        // Read actual velocity
        double actualVel1 = frontTransfer.getVelocity();


        // Telemetry
        dashboardTelemetry.addData("Target (ticks/s): ", targetVel);
        dashboardTelemetry.addData("Actual (ticks/s): ", actualVel1);
        dashboardTelemetry.addData("Encoder:", frontTransfer.getCurrentPosition());
        dashboardTelemetry.update();

    }
}

