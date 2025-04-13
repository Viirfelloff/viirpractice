package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.management.HardwareRobot;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Collections;

public class InDepSubsystem extends SubsystemBase {

    private final HardwareRobot HARDWARE_ROBOT;
    private final LinearOpMode OP_MODE;
    private final DriveSubsystem DRIVE;
    private final CVSubsystem CV;

    public final double ABV_INTAKE_ALIGN_TGT_X = 0.5;  // percentage of width
    public final double ABV_INTAKE_ALIGN_TGT_Y = 0.5;  // percentage of height
    public final double ARD_INTAKE_ALIGN_TGT_X = 0.5;  // percentage of width
    public final double ARD_INTAKE_ALIGN_TGT_Y = 0.5;  // percentage of height

    public final int ABV_INTAKE_ALIGN_ERR_THRESH_X = 6;  // in pixels
    public final int ABV_INTAKE_ALIGN_ERR_THRESH_Y = 6;  // in pixels
    public final int ARD_INTAKE_ALIGN_ERR_THRESH_X = 30;  // in pixels
    public final int ARD_INTAKE_ALIGN_ERR_THRESH_Y = 30;  // in pixels

    public InDepSubsystem(
            HardwareRobot hardwareRobot,
            LinearOpMode opMode,
            DriveSubsystem driveSubsystem,
            CVSubsystem cvSubsystem
    ) {
        HARDWARE_ROBOT = hardwareRobot;
        OP_MODE = opMode;
        DRIVE = driveSubsystem;
        CV = cvSubsystem;
    }


    //////////
    // CLAW //
    //////////
    // TODO: Tune all servo values
    public enum ClawPosition {
        CLOSED(0, "Closed"),
        PARTIAL(0.5, "Partial"),
        OPEN(1, "Open");

        public final double SERVO_POSITION;
        public final String NAME;

        private ClawPosition(double servoPosition, String name) {
            SERVO_POSITION = servoPosition;
            NAME = name;
        }
    }

    public void setClawPosition(ClawPosition position) {
        setClawPosition(position.SERVO_POSITION);
    }

    public void setClawPosition(double position) {
        HARDWARE_ROBOT.claw.setPosition(position);
    }

    ///////////
    // ELBOW //
    ///////////
    public enum ElbowPosition {
        CENTER(0.5, "Center"),
        LOWER_UPPER_CENTER(0.6, "Lower upper center"),
        UPPER_CENTER(0.75, "Upper center"),
        UP(1, "Up");

        public final double SERVO_POSITION;
        public final String NAME;

        private ElbowPosition(double servoPosition, String name) {
            SERVO_POSITION = servoPosition;
            NAME = name;
        }
    }

    public void setElbowPosition(ElbowPosition position) {
        setElbowPosition(position.SERVO_POSITION);
    }

    public void setElbowPosition(double position) {
        HARDWARE_ROBOT.elbow.setPosition(position);
    }

    //////////
    // LIFT //
    //////////
    public enum LiftPosition {
        LOW(0),
        ABV_SPC(5000),
        BLW_SPC(4000),
        BSK(7000),
        HANG(2500);

        public final int ENCODER_TICKS;

        LiftPosition(int encoderTicks) {
            ENCODER_TICKS = encoderTicks;
        }
    }

    public void setLiftPosition(LiftPosition position) {
        setLiftPosition(position.ENCODER_TICKS);
    }

    public void setLiftPosition(int position) {
        HARDWARE_ROBOT.leftLift.setTargetPosition(position);
        // ew, use synchropather liftplan pid instead

    }

    public int getLeftLiftPosition() {
        return HARDWARE_ROBOT.leftLift.getCurrentPosition();
    }

    public int getRightLiftPosition() {
        return HARDWARE_ROBOT.rightLift.getCurrentPosition();
    }

    public void setLeftLiftPower(double power) {
        HARDWARE_ROBOT.leftLift.motor.setPower(power);
    }

    public void setRightLiftPower(double power) {
        HARDWARE_ROBOT.rightLift.motor.setPower(power);
    }

    public void resetLiftEncoders() {
        HARDWARE_ROBOT.leftLift.resetEncoder();
        HARDWARE_ROBOT.rightLift.resetEncoder();
    }
}