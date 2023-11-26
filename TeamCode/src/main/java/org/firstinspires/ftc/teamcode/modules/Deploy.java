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
    private final Servo servo0;
    private final Servo servo1;
    private final double takePos = 0;
    private final double deployPos = 1;
    private final double closedPos = 0;
    private double closedPos2;
    private final double openPos = 1;

    public enum State {
        CLOSED,
        OPEN
    }
    public enum StateRotation {
        TAKE,
        DEPLOY
    }
    public State state = State.CLOSED;
    public StateRotation stateRotation = StateRotation.TAKE;

    public Deploy(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;

        servo0 = hardwareMap.get(Servo.class, "servo_rotation");
        servo1 = hardwareMap.get(Servo.class, "servo_hold");
        servo0.setDirection(Servo.Direction.FORWARD);
        servo1.setDirection(Servo.Direction.FORWARD);
    }

    public void tele() {
        if (gamepad.b) {
            switch (state) {
                case OPEN:
                    state = State.CLOSED;
                    servo1.setPosition(closedPos);
                case CLOSED:
                    state = State.OPEN;
                    servo1.setPosition(openPos);
            }
        }
        if (gamepad.a) {
            switch (stateRotation) {
                case TAKE:
                    servo0.setPosition(deployPos);
                    stateRotation = StateRotation.DEPLOY;
                case DEPLOY:
                    servo0.setPosition(takePos);
                    stateRotation = StateRotation.TAKE;
            }
        }

        telemetry.addData("rotation_servo: ", servo0.getPosition());
        telemetry.addData("hold servo: ", servo1.getPosition());
    }

    public void easyTele() {
        if (gamepad.dpad_up) {
            servo1.setPosition(0.38); // Open
        }
        if (gamepad.dpad_down) {
            servo1.setPosition(0.028); // Close
        }
        if (gamepad.y) {
            servo0.setPosition(0);
        }
        if (gamepad.a) {
            servo0.setPosition(0.4);
        }


        telemetry.addData("rotation_servo: ", servo0.getPosition());
        telemetry.addData("hold servo: ", servo1.getPosition());
    }

    public void testing() {
        telemetry.addData("Deploy: ", "rotation %4.2f", servo0.getPosition());
        telemetry.addData("Deploy: ", "deploy_servo1  %4.2f", servo1.getPosition());
    }
}
