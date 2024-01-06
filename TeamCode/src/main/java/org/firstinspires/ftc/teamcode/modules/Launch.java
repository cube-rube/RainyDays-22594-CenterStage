package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


// TODO: Сделать
public class Launch {
    private final LinearOpMode linearOpMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final Servo servoHold;
    private final Servo servoRaise;

    private float up_pos;
    private float down_pos;
    private double hold_pos;
    private double release_pos;



    public Launch(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;

        servoHold = hardwareMap.get(Servo.class, "servo_hold");
        servoRaise = hardwareMap.get(Servo.class, "servo_raise");

        telemetry.addData("Launch: ", "Initialized");

        hold_pos = 0;
        release_pos = 1;
    }

    public void tele() {
        if (gamepad.right_trigger > 0) {
            servoRaise.setPosition(up_pos);
        } else if (gamepad.left_trigger > 0) {
            servoRaise.setPosition(down_pos);
        }
        if (gamepad.right_bumper && gamepad.left_bumper) {
            servoHold.setPosition(release_pos);
        } else {
            servoHold.setPosition(hold_pos);
        }
    }

    public void teleWithPos() {
        servoRaise.setPosition(gamepad.right_stick_y);
        if (gamepad.b) {
            servoHold.setPosition(release_pos);
        }
        else {
            servoHold.setPosition(hold_pos);
        }
    }
}
