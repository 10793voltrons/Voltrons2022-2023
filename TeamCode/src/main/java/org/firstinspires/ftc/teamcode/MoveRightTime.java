package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "MoveRightTime", group = "Auto")
public class MoveRightTime extends LinearOpMode {

    // Primero declaramos todas las variables que vamos a usar
    // ( Motores, servos y temporizadores)

    DcMotor leftFront;
    DcMotor rightFront;

    DcMotor leftBack;
    DcMotor rightBack;

    DcMotor duckArm;
    DcMotor arm;

    Servo leftClaw;
    Servo rightClaw;

    public static double gyroKp = 0.003;
    //Servo ankle;

    public static double leftClawOpen = 0.25;
    public static double leftClawClosed = .65;

    public static double rightClawOpen = 0.35;
    public static double rightClawClosed = 0;

    public static double ankleDivider = 100;

    boolean open = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Luego las asignamos a su respectivo pedazo de hardware
        leftFront = hardwareMap.dcMotor.get("fl");
        rightFront = hardwareMap.dcMotor.get("fr");
        leftBack = hardwareMap.dcMotor.get("bl");
        rightBack = hardwareMap.dcMotor.get("br");

        duckArm = hardwareMap.dcMotor.get("duck");
        arm = hardwareMap.dcMotor.get("arm");

        leftClaw = hardwareMap.servo.get("lcl");
        rightClaw = hardwareMap.servo.get("rcl");

        //ankle = hardwareMap.servo.get("ankle");

        // Invertimos los motores de frabrica
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        // Hacemos esto para que por defecto, cuando alguien deje de mover el stick de motor, se frenen todos los motores y no se quede patinando
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Imu
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        double invert = 1;
        double adjust = 10;
        double k_p = 0.000;

        double position_goal = arm.getCurrentPosition();
        double anklePosition = 0.39;

        leftClaw.setPosition(leftClawOpen);
        rightClaw.setPosition(rightClawOpen);

        Drive drive = new Drive(leftFront, leftBack, rightFront, rightBack, arm, imu);

        //Esto agregue
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //OpenCvCamera phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //
        waitForStart();

        if (opModeIsActive()) {

            sleep(1000);


            sleep(2000);
            //drive.frontTimed(0.3, 2600, 220, gyroKp);
            drive.rightTimed(0.3, 1000, 180, gyroKp);
            sleep(1500);
            //drive.setOrientation(0.3, 60, 1080);
            //sleep(2000);

            //drive.frontTimed(0.3 , 1500, 90, gyroKp);

            sleep(1000);

            telemetry.addData("Duck Power", duckArm.getPower());
            telemetry.addData("Invert", invert);
            telemetry.addData("Left Claw Position", leftClaw.getPosition());
            telemetry.addData("Right Claw Position", rightClaw.getPosition());
            //telemetry.addData("Ankle", ankle.getPosition());
            //telemetry.addData("Error", error);
            //telemetry.addData("Arm Power", armPower);
            telemetry.addData("Arm position", arm.getCurrentPosition());
            telemetry.addData("Position Goal", position_goal);
            telemetry.addData("Slow Mode", adjust == 4);
            telemetry.update();

        }
    }
}
