package org.firstinspires.ftc.teamcode.management;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;

public class HardwareRobot {
    public final MotorEx leftFront;
    public final MotorEx rightFront;
    public final MotorEx leftBack;
    public final MotorEx rightBack;

    public final MotorEx leftLift;
    public final MotorEx rightLift;

    public final GoBildaPinpointDriver pinpoint;

    public final Limelight3A limelight;
    public final WebcamName cameraName;

    public final Servo elbow;
    public final Servo claw;  // claw open/close servo
    public final Servo wrist;


    public HardwareRobot(HardwareMap hardwareMap) {

        ////////////
        // WHEELS //
        ////////////
        leftFront = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
        rightFront = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
        leftBack = new MotorEx(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
        rightBack = new MotorEx(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);

        leftFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setRunMode(Motor.RunMode.RawPower);
        rightFront.setRunMode(Motor.RunMode.RawPower);
        leftBack.setRunMode(Motor.RunMode.RawPower);
        rightBack.setRunMode(Motor.RunMode.RawPower);

        leftFront.setInverted(true);
        rightFront.setInverted(true);
        leftBack.setInverted(true);
        rightBack.setInverted(true);

        leftFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //////////
        // LIFT //
        //////////

        leftLift = new MotorEx(hardwareMap, "leftLift", Motor.GoBILDA.RPM_312);
        leftLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setRunMode(Motor.RunMode.RawPower);
        leftLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftLift.setInverted(false);

        rightLift = new MotorEx(hardwareMap, "rightLift", Motor.GoBILDA.RPM_312);
        rightLift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setRunMode(Motor.RunMode.RawPower);
        rightLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightLift.setInverted(true);


        //////////////
        // PINPOINT //
        //////////////
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        ////////////
        // SERVOS //
        ////////////
        elbow = hardwareMap.get(ServoImplEx.class, "elbow");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");


        ////////////
        // CAMERA //
        ////////////
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

    }
}