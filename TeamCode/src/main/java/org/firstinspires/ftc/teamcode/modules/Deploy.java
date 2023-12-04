package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class Deploy {
    private final LinearOpMode linearOpMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final Servo servoRotation;
    private final Servo servoHold;

    public enum HolderState {
        HOLD,
        RELEASE
    }
    private final static double holdPos = 0;
    private final static double releasePos = 1;
    public enum RotationState {
        TAKE,
        DEPLOY
    }
    private final static double takePos = 0;
    private final static double deployPos = 1;
    private enum ButtonState {
        PRESSED,
        HELD,
        RELEASED
    }
    private ButtonState aState = ButtonState.RELEASED;
    private ButtonState yState = ButtonState.RELEASED;
    public HolderState holderState = HolderState.RELEASE;
    public RotationState rotationState = RotationState.TAKE;

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
        switch (aState) {
            case PRESSED:
                switch (holderState) {
                    case RELEASE:
                        holderState = HolderState.HOLD;
                    case HOLD:
                        holderState = HolderState.RELEASE;
                }
                if (gamepad.a) {
                    aState = ButtonState.HELD;
                } else {
                    aState = ButtonState.RELEASED;
                }
            case HELD:
                if (!gamepad.a) {
                    aState = ButtonState.RELEASED;
                }
            case RELEASED:
                if (gamepad.a) {
                    aState = ButtonState.PRESSED;
                }
        }
        switch (yState) {
            case PRESSED:
                switch (rotationState) {
                    case TAKE:
                        rotationState = RotationState.DEPLOY;
                    case DEPLOY:
                        rotationState = RotationState.TAKE;
                }
                if (gamepad.a) {
                    yState = ButtonState.HELD;
                } else {
                    yState = ButtonState.RELEASED;
                }
            case HELD:
                if (!gamepad.a) {
                    yState = ButtonState.RELEASED;
                }
            case RELEASED:
                if (gamepad.a) {
                    yState = ButtonState.PRESSED;
                }
        }
        switch (holderState) {
            case HOLD:
                servoHold.setPosition(holdPos);
            case RELEASE:
                servoHold.setPosition(releasePos);
        }
        switch (rotationState) {
            case TAKE:
                servoRotation.setPosition(takePos);
            case DEPLOY:
                servoRotation.setPosition(deployPos);
        }
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
