package fsmTests.mathTests;

import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.junit.jupiter.MockitoExtension;

import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.misc.gamepad.Toggle;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.vision.logi;
import org.firstinspires.ftc.teamcode.teleop.fsm.FSM;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.Spy;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@ExtendWith(MockitoExtension.class)
public class DynamicShootingTests {

    // 4 TESTS

    // -- SHOOTER --
    @Mock
    DcMotorEx outtake1, outtake2;

    @Mock
    Servo hood;

    Shooter shooter;

    @Mock
    logi webCam;

    // -- INTAKE --
    @Mock
    DcMotorEx intake;

    // -- TRANSFER --
    @Mock
    CRServo frontTransfer, backTransfer;

    // -- DRIVETRAIN --
    @Mock
    DcMotorEx leftFront;
    @Mock
    DcMotorEx rightFront;
    @Mock
    DcMotorEx leftBack;
    @Mock
    DcMotorEx rightBack;
    @Mock
    IMU imu;

    // -- OTHER HARDWARE --
    Gamepad gamepad1 = new Gamepad();
    Gamepad gamepad2 = new Gamepad();

    @Mock
    // uh this prob wont run so use the interface or spy it if you need it
    GoBildaPinpointDriver pinpoint;

    @Mock
    DigitalChannel led0, led1;


    // -- ACTUAL OBJECTS --
    Intake intakeMech;
    Drivetrain drivetrain;
    Transfer transferMech;

    GamepadMapping controls = new GamepadMapping(gamepad1, gamepad2);
    Robot robot;

    @BeforeEach
    public void setUp() {
        shooter = new Shooter(outtake1, outtake2, hood);
        intakeMech = new Intake(intake);
        drivetrain = new Drivetrain(leftFront, rightFront, leftBack, rightBack, imu);
        transferMech = new Transfer(frontTransfer, backTransfer);

        robot = new Robot(controls, imu, pinpoint, webCam, intakeMech, transferMech,
                shooter, drivetrain, led0, led1);
    }


    @Test
    public void testShooterConversion() {
        when(webCam.getATdist()).thenReturn(48.0);
        robot.cam = webCam;

        double actualMPS = shooter.calculateShooterMPS(robot.cam.getATdist());

        double actualRPM = shooter.calculateShooterRPM(actualMPS);

        assertEquals(actualRPM, 622.167071180941);
    }

    @Test
    public void testShooterCalculation48() {
        // 48 inches from the goal
        when(webCam.getATdist()).thenReturn(48.0);
        robot.cam = webCam;

        double actualMPS = shooter.calculateShooterMPS(robot.cam.getATdist());

        assertEquals(3.99812243100862, actualMPS);
    }

    @Test
    public void testShooterCalculation100() {
        // 48 inches from the goal
        when(webCam.getATdist()).thenReturn(100.0);
        robot.cam = webCam;

        double actualMPS = shooter.calculateShooterMPS(robot.cam.getATdist());

        assertEquals(5.353239674370232, actualMPS);
    }

    @Test
    public void testHoodCalculation() {
        // 48 inches from the goal
        when(webCam.getATdist()).thenReturn(48.0);
        robot.cam = webCam;

        double hoodPos = shooter.calculateHoodPos(robot.cam.getATdist());

        assertEquals(0.37663332819326245, hoodPos);
    }
}
