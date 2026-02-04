package org.firstinspires.ftc.teamcode.subsystems.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.park.Park;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.transfer.TransferM;
import org.firstinspires.ftc.teamcode.subsystems.vision.logi;

public class Robot {
// CONFIG
    // right - expansion
    // left - control
    // front - 0
    // back - 1


    // servo variableHood    control hub 1
    // servo transferB       control hub 2

    // servo park            expansion hub 5

    // motor outtakeTop      control hub 2
    // motor outtakeBot      control hub 3

    // motor transferF       expansion hub port 2
    // motor intake          expansion hub port 3

    // MECHANISMS
    public final IMU imu;
    public Intake intake;
    //public Transfer transfer;
    public TransferM transfer;
    public Shooter shooter;
    public Drivetrain drivetrain;

    public Park park;


    public GoBildaPinpointDriver driver;

    public GamepadMapping controls;

    public static logi cam;

    public DigitalChannel ledBoard0;
    public DigitalChannel ledBoard1;

    public Robot(HardwareMap hardwareMap, GamepadMapping controls) {
        this.controls = controls;

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        cam = new logi(hardwareMap);

        intake = new Intake(hardwareMap);
        // transfer = new Transfer(hardwareMap);
        transfer = new TransferM(hardwareMap);
        shooter = new Shooter(hardwareMap);
        park = new Park(hardwareMap);

        park = new Park(hardwareMap);

        drivetrain = new Drivetrain(hardwareMap, imu, controls);

        ledBoard0 = hardwareMap.get(DigitalChannel.class, "ledBoard0");
        ledBoard0.setMode(DigitalChannel.Mode.OUTPUT);
        ledBoard1 = hardwareMap.get(DigitalChannel.class, "ledBoard1");
        ledBoard1.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public Robot(GamepadMapping controls, IMU imu, GoBildaPinpointDriver pinpoint,
                 logi cam, Intake intake, TransferM transfer, Shooter shooter, Drivetrain dt,
                 DigitalChannel led0, DigitalChannel led1, Park park) {
        this.controls = controls;
        this.imu = imu;
        this.driver = pinpoint;
        // this.cam = cam;
        this.intake = intake;
        this.transfer = transfer;
        this.shooter = shooter;
        this.drivetrain = dt;
        this.ledBoard0 = led0;
        this.ledBoard1 = led1;
        this.park = park;
    }

    public void hardwareSoftReset() {
        transfer.transferOff();
        shooter.hoodToBack();
        intake.intakeOff();
        shooter.setShooterVelocity(0);
    }
}
