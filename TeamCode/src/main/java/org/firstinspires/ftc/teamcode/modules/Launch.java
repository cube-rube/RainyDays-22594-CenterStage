package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launch {
    private LinearOpMode linearOpMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private Servo servoHold, servoRaise;

    private float upPos, downPos, holdPos, releasePos;

    public Launch(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;

        servoHold = hardwareMap.get(Servo.class, "servo_hold");
        servoRaise = hardwareMap.get(Servo.class, "servo_raise");

        telemetry.addData("Claw: ", "Initialized");
    }

    public void tele() {
        if (gamepad.right_trigger > 0) {
            servoRaise.setPosition(upPos);
        } else if (gamepad.left_trigger > 0) {
            servoRaise.setPosition(downPos);
        }
        if (gamepad.right_bumper && gamepad.left_bumper) {
            servoHold.setPosition(releasePos);
        }
        else {
            servoHold.setPosition(holdPos);
        }
    }
}
