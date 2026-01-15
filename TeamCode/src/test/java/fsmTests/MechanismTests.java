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
        
    }

    @Test
    public void shootFromFront() {

    }

    @Test
    public void setVelocity() {

    }

    @Test
    public void setHoodVelocity() {

    }

    @Test
    public void setHoodFront() {

    }

    @Test
    public void setHoodBack() {

    }

    // -- INTAKE --
    @Test
    public void intakeOn() {

    }

    @Test
    public void intakeOff() {

    }

    @Test
    public void intakeReverse() {

    }

    // -- TRANSFER --
    @Test
    public void transferOn() {

    }

    @Test
    public void transferOff() {

    }

    @Test
    public void hotDog() {

    }
}
