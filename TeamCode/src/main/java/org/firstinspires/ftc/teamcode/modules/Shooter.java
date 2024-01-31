package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.utility.ButtonState;


// TODO: Сделать
public class Shooter {
    private final LinearOpMode linearOpMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final Servo servoHold;
    private final Servo servoRaise;

    private double up_pos = 0.15;
    private double up_pos1 = 0.04;
    private double down_pos = 0;
    public static double hold_pos = 0.55;
    public static double release_pos = 1;
    private ButtonState stickLeftState = ButtonState.RELEASED;
    private ButtonState stickRightState = ButtonState.RELEASED;



    public Shooter(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;

        servoHold = hardwareMap.get(Servo.class, "servo_launch");
        servoRaise = hardwareMap.get(Servo.class, "servo_angle");

        telemetry.addLine("Shooter: Initialized");
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
        switch (stickRightState) {
            case PRESSED:
                servoRaise.setPosition(servoRaise.getPosition() + 0.05);
                if (gamepad.right_stick_button) {
                    stickRightState = ButtonState.HELD;
                } else {
                    stickRightState = ButtonState.RELEASED;
                }
                break;
            case HELD:
                if (!gamepad.right_stick_button) {
                    stickRightState = ButtonState.RELEASED;
                }
                break;
            case RELEASED:
                if (gamepad.right_stick_button) {
                    stickRightState = ButtonState.PRESSED;
                }
                break;
        }
        switch (stickLeftState) {
            case PRESSED:
                servoRaise.setPosition(servoRaise.getPosition() - 0.05);
                if (gamepad.left_stick_button) {
                    stickLeftState = ButtonState.HELD;
                } else {
                    stickLeftState = ButtonState.RELEASED;
                }
                break;
            case HELD:
                if (!gamepad.left_stick_button) {
                    stickLeftState = ButtonState.RELEASED;
                }
                break;
            case RELEASED:
                if (gamepad.left_stick_button) {
                    stickLeftState = ButtonState.PRESSED;
                }
                break;
        }
        if (gamepad.left_trigger > 0 && gamepad.left_bumper) {
            servoHold.setPosition(release_pos);
        }
        else {
            servoHold.setPosition(hold_pos);
        }
    }
}
