package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.MainTeleOp;
import org.opencv.core.Mat;


@Config
public class Intake {
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final DcMotor motor;
    private final DigitalChannel sensor;
    private final Servo leftServo;
    private final Servo rightServo;
    public static double TPR = 292.1212;

    public enum Direction {
        FORWARD,
        F_STOP,
        B_STOP,
        BACKWARD,
    }
    private Direction direction = Direction.F_STOP;
    public enum IntakeState {
        EJECT,
        INTAKE,
        BREAK,
        STOP
    }
    public IntakeState intakeState = IntakeState.STOP;
    public static double STOP_POWER = 0.15;
    public static double INTAKE_POWER = -0.8;
    public static double OUT_POWER = 0.6;
    public static double OPEN_LEFT_POS = 0.03;
    public static double OPEN_RIGHT_POS = 0.65;
    public static double CLOSE_LEFT_POS = 0.75;
    public static double CLOSE_RIGHT_POS = 0;

    public Intake(LinearOpMode linearOpMode) {
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad1;

        telemetry.addLine("Intake: initializing motor");
        motor = hardwareMap.get(DcMotor.class, "motor_intake");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Intake: initializing sensor");
        sensor = hardwareMap.get(DigitalChannel.class, "sensor_intake");
        sensor.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addLine("Intake: initializing servos");
        leftServo = hardwareMap.get(Servo.class, "left_flap");
        rightServo = hardwareMap.get(Servo.class, "right_flap");

        telemetry.addLine("Intake: Initialized");
    }

    public void opControlOld() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (gamepad.y) {
            motor.setPower(-0.6);
        } else if (gamepad.a) {
            motor.setPower(0.85);
        } else {
            motor.setPower(0);
        }
    }
    public void opControlSensor() {
        if (gamepad.y) {
            motor.setPower(OUT_POWER);
            direction = Direction.BACKWARD;
        } else if (gamepad.a) {
            motor.setPower(INTAKE_POWER);
            direction = Direction.BACKWARD;
        } else {
            switch (direction) {
                case FORWARD: motor.setPower(STOP_POWER);
                    break;
                case BACKWARD: motor.setPower(-STOP_POWER);
                    break;
                case B_STOP:
                case F_STOP:
                    motor.setPower(0);
                    break;
            }
            if (!sensor.getState()) {
                switch (direction) {
                    case FORWARD: direction = Direction.F_STOP;
                        break;
                    case BACKWARD: direction = Direction.B_STOP;
                        break;
                }
            } else {
                switch (direction) {
                    case B_STOP: direction = Direction.FORWARD;
                        break;
                    case F_STOP: direction = Direction.BACKWARD;
                        break;
                }
            }
        }
        telemetry.addLine("---------------");
        telemetry.addLine("Intake:");
        telemetry.addData("intake_pos", motor.getCurrentPosition());
        telemetry.addData("direction", direction);
        telemetry.addData("sensor_state", !sensor.getState());
    }

    public void take() {
        intakeState = IntakeState.INTAKE;
    }

    public void ejectOp() {
        motor.setPower(OUT_POWER);
        direction = Direction.BACKWARD;
    }

    public void eject() {
        intakeState = IntakeState.EJECT;
    }
    public void break_() {
        intakeState = IntakeState.BREAK;
    }

    public void stop() {
        intakeState = IntakeState.STOP;
    }

    public boolean isMoving() {
        return motor.getPower() != 0;
    }

    public void openLeftFlap() {
        leftServo.setPosition(OPEN_LEFT_POS);
    }

    public void openRightFlap() {
        rightServo.setPosition(OPEN_RIGHT_POS);
    }

    public void closeLeftFlap() {
        leftServo.setPosition(CLOSE_LEFT_POS);
    }

    public void closeRightFlap() {
        rightServo.setPosition(CLOSE_RIGHT_POS);
    }

    public void autoControl() {
        switch (intakeState) {
            case INTAKE:
                motor.setPower(-0.7);
                direction = Direction.BACKWARD;
                break;
            case EJECT:
                motor.setPower(0.7);
                direction = Direction.BACKWARD;
                break;
            case BREAK:
                motor.setPower(0.5);
                direction = Direction.BACKWARD;
                break;
            case STOP:
                switch (direction) {
                    case FORWARD:
                        motor.setPower(STOP_POWER);
                        break;
                    case BACKWARD:
                        motor.setPower(-STOP_POWER);
                        break;
                    case B_STOP:
                    case F_STOP:
                        motor.setPower(0);
                        break;
                }
                if (!sensor.getState()) {
                    switch (direction) {
                        case FORWARD:
                            direction = Direction.F_STOP;
                            break;
                        case BACKWARD:
                            direction = Direction.B_STOP;
                            break;
                    }
                } else {
                    switch (direction) {
                        case B_STOP:
                            direction = Direction.FORWARD;
                            break;
                        case F_STOP:
                            direction = Direction.BACKWARD;
                            break;
                    }
                }
                break;
        }
    }

    public void testingSensor() {
        telemetry.addData("state", sensor.getState());
        telemetry.addData("string", sensor.toString());
        telemetry.addData("device", sensor.getDeviceName());
        telemetry.addData("manufacturer", sensor.getManufacturer());
        telemetry.addData("connection", sensor.getConnectionInfo());
    }

    public void setPower(double n) {
        motor.setPower(n);
    }

    public void testing() {
        if (gamepad.dpad_down) {
            motor.setPower(0.1);
        } else if (gamepad.dpad_up) {
            motor.setPower(-0.1);
        } else {
            motor.setPower(0);
        }

        telemetry.addData("intake_motor_pos", motor.getCurrentPosition());
        telemetry.addData("intake_motor_power", motor.getPowerFloat());
    }
}
