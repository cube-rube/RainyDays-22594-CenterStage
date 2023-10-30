package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Deploy {
    private LinearOpMode linearOpMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private Servo servo0, servo1;
    private double takePos;
    private double deployPos;
    private double closedPos1, closedPos2;
    private double closingDelta;

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

        servo0 = hardwareMap.get(Servo.class, "rotation_servo");
        servo1 = hardwareMap.get(Servo.class, "deploy_servo1");
        servo0.setDirection(Servo.Direction.REVERSE);
        servo1.setDirection(Servo.Direction.FORWARD);

        takePos = servo0.getPosition();
        closedPos1 = servo1.getPosition();
        telemetry.addData("Deploy: TakePos ", "%4.2f", takePos);
    }

    public void tele() {
        if (gamepad.b) {
            switch (state) {
                case OPEN:
                    state = State.CLOSED;
                    servo1.setPosition(closedPos1);
                case CLOSED:
                    state = State.OPEN;
                    servo1.setPosition(closedPos1 + closingDelta);
            }
        }
        if (gamepad.a) {
            switch (stateRotation) {
                case TAKE:
                    servo0.setPosition(deployPos);
                case DEPLOY:
                    servo0.setPosition(takePos);
            }
        }

        telemetry.addData("Deploy: rotation_servo ", "%4.2f", servo0.getPosition());
        telemetry.addData("Deploy: ", "deploy_servo1  %4.2f", servo1.getPosition());
    }

    public void testing() {
        telemetry.addData("Deploy: ", "rotation %4.2f", servo0.getPosition());
        telemetry.addData("Deploy: ", "deploy_servo1  %4.2f", servo1.getPosition());
    }
}
