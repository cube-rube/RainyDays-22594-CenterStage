package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


// TODO: Переделать
public class Deploy {
    private final LinearOpMode linearOpMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final Servo servoRotation;
    private final Servo servoHold;
    private final double takePos = 0;
    private final double deployPos = 1;
    private final double closedPos = 0;
    private final double openPos = 1;

    public enum State {
        CLOSED,
        OPEN
    }
    public enum StateRotation {
        TAKE,
        DEPLOY
    }

    private enum ButtonState {
        PRESSED,
        HELD,
        RELEASED
    }

    public State state = State.CLOSED;
    public StateRotation stateRotation = StateRotation.TAKE;

    public Deploy(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;

        servoRotation = hardwareMap.get(Servo.class, "servo_rotation");
        servoHold = hardwareMap.get(Servo.class, "servo_hold");
        servoRotation.setDirection(Servo.Direction.FORWARD);
        servoHold.setDirection(Servo.Direction.FORWARD);
    }

    public void tele() {
        if (gamepad.b) {
            switch (state) {
                case OPEN:
                    state = State.CLOSED;
                    servoHold.setPosition(closedPos);
                case CLOSED:
                    state = State.OPEN;
                    servoHold.setPosition(openPos);
            }
        }
        if (gamepad.a) {
            switch (stateRotation) {
                case TAKE:
                    servoRotation.setPosition(deployPos);
                    stateRotation = StateRotation.DEPLOY;
                case DEPLOY:
                    servoRotation.setPosition(takePos);
                    stateRotation = StateRotation.TAKE;
            }
        }

        telemetry.addData("rotation_servo: ", servoRotation.getPosition());
        telemetry.addData("hold servo: ", servoHold.getPosition());
    }

    public void easyTele() {
        if (gamepad.dpad_up) {
            servoHold.setPosition(0.38); // Open
        }
        if (gamepad.dpad_down) {
            servoHold.setPosition(0.028); // Close
        }
        if (gamepad.y) {
            servoRotation.setPosition(0);
        }
        if (gamepad.a) {
            servoRotation.setPosition(0.4);
        }


        telemetry.addData("rotation_servo: ", servoRotation.getPosition());
        telemetry.addData("hold servo: ", servoHold.getPosition());
    }

    public void testing() {
        telemetry.addData("Deploy: ", "rotation %4.2f", servoRotation.getPosition());
        telemetry.addData("Deploy: ", "deploy_servo1  %4.2f", servoHold.getPosition());
    }
}
