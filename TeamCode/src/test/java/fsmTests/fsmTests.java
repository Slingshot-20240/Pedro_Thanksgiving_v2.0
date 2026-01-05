package fsmTests;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
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

@ExtendWith(MockitoExtension.class)
public class fsmTests {
    // intake hardware
    @Mock
    DcMotorEx intake;

    // shooter hardware
    @Mock
    DcMotorEx outtake1, outtake2;
    @Mock
    Servo hood;

    // transfer hardware
    @Mock
    CRServo frontTransfer, backTransfer;

    // drivetrain hardware
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

    // other hardware
    Gamepad gamepad1 = new Gamepad();
    Gamepad gamepad2 = new Gamepad();

    @Mock
    // uh this prob wont run so use the interface or spy it if you need it
    GoBildaPinpointDriver pinpoint;

    @Mock
    DigitalChannel led0, led1;

//    @Mock
//    WebcamName webcam;

    // -- actual objects --
    Intake intakeMech;
    Shooter shooterMech;
    Drivetrain drivetrain;
    Transfer transferMech;
    logi cam;

    // this may not work...
    GamepadMapping controls = new GamepadMapping(gamepad1, gamepad2);
    FSM fsm;
    Robot robot;

    @BeforeEach
    public void setUp() {
        intakeMech = new Intake(intake);
        shooterMech = new Shooter(outtake1, outtake2, hood);
        drivetrain = new Drivetrain(leftFront, rightFront, leftBack, rightBack, imu);
        transferMech = new Transfer(frontTransfer, backTransfer);

        // cam = new logi(webcam);

        robot = new Robot(controls, imu, pinpoint, cam, intakeMech, transferMech,
                shooterMech, drivetrain, led0, led1);
        fsm = new FSM(null, controls, robot);
    }

    // This test is broken and stupid and dumb excuse my language
//    @Test
//    public void testBaseToOuttake() {
//        fsm.setControlType(FSM.ControlType.HARDCODED_CONTROL);
//        fsm.setState(FSM.FSMStates.BASE_STATE);
//        controls.outtake.setLocked(true);
//        fsm.update();
//        controls.outtake.setLocked(true);
//        fsm.update();
//        assertEquals(FSM.FSMStates.OUTTAKING, fsm.getState());
//    }

    @Test
    public void testOuttakeToBase() {
        fsm.setState(FSM.FSMStates.OUTTAKING);
        controls.outtake.setLocked(false);
        fsm.update();
        assertEquals(FSM.FSMStates.BASE_STATE, fsm.getState());
        fsm.update();
        verify(intake).setPower(-anyDouble());
    }

    @Test
    public void testBaseToHardcodedBack() {
        fsm.setControlType(FSM.ControlType.HARDCODED_CONTROL);
        fsm.setState(FSM.FSMStates.BASE_STATE);
        controls.shootBack.setLocked(true);
        fsm.update();
        assertEquals(FSM.FSMStates.SHOOT_BACK, fsm.getState());
        fsm.update();
        verify(outtake1).setPower(-anyDouble());
        verify(outtake2).setPower(-anyDouble());
    }

    @Test
    public void testHardcodedBackToBase() {

    }

    @Test
    public void testBaseToHardcodedFront() {

    }

    @Test
    public void testHardcodedFrontToBase() {

    }

    @Test
    public void baseIntakeOn() {

    }

    @Test
    public void baseIntakeOff() {

    }

    @Test
    public void baseToPIDShoot() {

    }

    @Test
    public void testPIDShootToBase() {

    }

    @Test
    public void baseLEDATDetected() {

    }

    @Test
    public void baseLEDATUndetected() {

    }
}
