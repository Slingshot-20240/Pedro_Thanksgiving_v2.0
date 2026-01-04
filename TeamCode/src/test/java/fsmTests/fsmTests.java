package fsmTests;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
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
    Servo rightExtendo;
    @Mock
    Servo leftExtendo;
    @Mock
    DcMotorEx rollerMotor;
    @Mock
    Servo pivotAxon;

    // outtake hardware
    @Mock
    DcMotorEx slideRight;
    @Mock
    DcMotorEx slideLeft;
    @Mock
    Servo bucketServo;
    PIDController controller = new PIDController(0, 0, 0);

    // arm
    @Mock
    Servo armPivot;
    @Mock
    Servo wrist;
    @Mock
    Servo claw;

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
    @Mock
    ColorRangeSensor colorSensor;
    @Mock
    TouchSensor touchSensor;

    ColorSensorI2C colorSensorI2C;

    // other hardware
    Gamepad gamepad1 = new Gamepad();
    Gamepad gamepad2 = new Gamepad();

    // -- actual objects --
    ActiveIntake activeIntake;
    Intake intake;
    Outtake outtake;
    DriveTrain drivetrain;
    Arm arm;

    // this may not work...
    GamepadMapping controls = new GamepadMapping(gamepad1, gamepad2);
    ActiveCycle cycle;
    Robot robot;
    ElapsedTime loopTime;
    double startTime;
}
