package fsmTests;

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
public class MechanismTests {
    // -- INTAKE --
    @Mock
    DcMotorEx intake;

    // -- SHOOTER --
    @Mock
    DcMotorEx outtake1, outtake2;
    @Mock
    Servo hood;

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

    @Mock
    logi webCam;


    // -- ACTUAL OBJECTS --
    Intake intakeMech;
    Shooter shooterMech;
    Drivetrain drivetrain;
    Transfer transferMech;

    // this may not work...
    GamepadMapping controls = new GamepadMapping(gamepad1, gamepad2);
    Robot robot;

    @BeforeEach
    public void setUp() {
        intakeMech = new Intake(intake);
        shooterMech = new Shooter(outtake1, outtake2, hood);
        drivetrain = new Drivetrain(leftFront, rightFront, leftBack, rightBack, imu);
        transferMech = new Transfer(frontTransfer, backTransfer);

        robot = new Robot(controls, imu, pinpoint, webCam, intakeMech, transferMech,
                shooterMech, drivetrain, led0, led1);
    }

    // 12 TESTS

    // -- SHOOTER --
    @Test
    public void shootFromBack() {
        shooterMech.shootFromBack();

        verify(outtake1).setVelocity(anyDouble());
        verify(outtake2).setVelocity(anyDouble());
    }

    @Test
    public void shootFromFront() {
        shooterMech.shootFromFront();

        verify(outtake1).setVelocity(anyDouble());
        verify(outtake2).setVelocity(anyDouble());
    }

    @Test
    public void setVelocity() {
        shooterMech.setShooterVelocity(1000);

        verify(outtake1).setVelocity(anyDouble());
    }

    @Test
    public void setHoodAngle() {
        shooterMech.setHoodAngle(60);

        verify(hood).setPosition(anyDouble());
    }

    @Test
    public void setHoodFront() {
        shooterMech.setHoodAngle(.5);

        verify(hood).setPosition(.5);
    }

    @Test
    public void setHoodBack() {
        shooterMech.setHoodAngle(.175);

        verify(hood).setPosition(.175);
    }

    // -- INTAKE --
    @Test
    public void intakeOn() {
        intakeMech.intakeOn();

        verify(intake).setPower(anyDouble());
    }

    @Test
    public void intakeOff() {
        intakeMech.intakeOff();

        verify(intake).setPower(0);
    }

    @Test
    public void intakeReverse() {
        intakeMech.intakeReverse();

        verify(intake).setPower(-1);
    }

    // -- TRANSFER --
    @Test
    public void transferOn() {
        transferMech.transferOn();

        verify(backTransfer).setPower(anyDouble());
        verify(frontTransfer).setPower(anyDouble());
    }

    @Test
    public void transferOff() {
        transferMech.transferOff();

        verify(backTransfer).setPower(0);
        verify(frontTransfer).setPower(0);
    }

    @Test
    public void hotDog() {
        transferMech.hotDog();

        verify(backTransfer).setPower(1);
        verify(frontTransfer).setPower(-.12);
    }
}
