package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launch {
    private final LinearOpMode linearOpMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final Servo servoHold;
    private final Servo servoRaise;

    private float upPos;
    private float downPos;
    private double holdPos;
    private double releasePos;



    public Launch(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;

        servoHold = hardwareMap.get(Servo.class, "servo_hold");
        servoRaise = hardwareMap.get(Servo.class, "servo_raise");

        telemetry.addData("Launch: ", "Initialized");

        holdPos = 0;
        releasePos = 1;
    }

    /*public void tele() {
        if (gamepad.right_trigger > 0) {
            servoRaise.setPosition(upPos);
        } else if (gamepad.left_trigger > 0) {
            servoRaise.setPosition(downPos);
        }
        if (gamepad.right_bumper && gamepad.left_bumper) {
            servoHold.setPosition(releasePos);
        } else {
            servoHold.setPosition(holdPos);
        }
    }*/

    public void  tele() {
        servoRaise.setPosition(gamepad.right_stick_y);
        if (gamepad.b) {
            servoHold.setPosition(releasePos);
        }
        else {
            servoHold.setPosition(holdPos);
        }
    }
}
