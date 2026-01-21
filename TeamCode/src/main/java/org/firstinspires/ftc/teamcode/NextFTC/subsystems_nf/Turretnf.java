package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
public class Turretnf implements Subsystem {

    public static double distance;
    public static PIDCoefficients turretPIDCoefficients =
            new PIDCoefficients(0.005, 0.0, 0);

    public static final Turretnf INSTANCE = new Turretnf();
    private Turretnf() {}

    private CRServoEx turretServoL = new CRServoEx("tL", 0.03);
    private CRServoEx turretServoR = new CRServoEx("tR", 0.03);

    public AnalogInput encoderL;
    public AnalogInput encoderR;


    public ControlSystem turretPIDController = ControlSystem.builder()
            .posPid(turretPIDCoefficients)
            .build();


    public static Double targetTurretAng = 0.0;
    private static double offset = 4.0;
    private static final double GEAR_RATIO = 3.25;
    private static final double TURRET_MIN_DEG = -85;
    private static final double TURRET_MAX_DEG =  85;
    public static boolean AUTO_AIM = true;

    Pose goal = new Pose(144,144);


    @Override
    public void initialize() {
        encoderL = ActiveOpMode.hardwareMap().get(AnalogInput.class, "tLe");
        encoderR = ActiveOpMode.hardwareMap().get(AnalogInput.class, "tRe");
    }

    @Override
    public void periodic() {

        if (AUTO_AIM) {
            turretLoop();
        }
        distanceFunc();
        turretPIDController.setGoal(new KineticState(targetTurretAng));
        double power = turretPIDController.calculate(new KineticState(getTurretDegrees()));

        turretServoL.setPower(power);
        turretServoR.setPower(power);

    }

    public void turretLoop(){

        double dx = goal.getX() - PedroComponent.follower().getPose().getX();
        double dy = goal.getY() - PedroComponent.follower().getPose().getY();

        double goalFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
        double headingDeg   = Math.toDegrees(PedroComponent.follower().getHeading());

        // turret relative to robot forward 0 degree facing to goal side
        double desired = normalizeAngle(goalFieldDeg - headingDeg);
        targetTurretAng = clamp(desired, TURRET_MIN_DEG, TURRET_MAX_DEG);

    }
    public void velocityBasedTurret(){
        Vector robot2Goal = new Vector(
                144-PedroComponent.follower().getPose().getX(),
                144-PedroComponent.follower().getPose().getY()
        );
        Vector velocity = PedroComponent.follower().getVelocity();
        double heading = PedroComponent.follower().getHeading();
        double coordTheta = velocity.getTheta() - robot2Goal.getTheta();
        double perpComponent = Math.sin(coordTheta) * velocity.getMagnitude();

        distance = Math.hypot(144-PedroComponent.follower().getPose().getX(), 144-PedroComponent.follower().getPose().getY());
    }


    private double distanceFunc(){
        return distance = Math.hypot(goal.getX()-PedroComponent.follower().getPose().getX(), goal.getY()-PedroComponent.follower().getPose().getY());
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    public double normalizeAngle(double angDeg) {
        double ang = angDeg;
        if (ang < 0.0) ang += 360.0;
        if (ang > 180.0) ang -= 360.0;
        return ang;
    }


    private double getTurretDegrees() {
        double servoDeg = (encoderR.getVoltage() / 3.3) * 360.0;

        // convert to turret degrees
        return normalizeAngle(servoDeg / GEAR_RATIO);
    }

}