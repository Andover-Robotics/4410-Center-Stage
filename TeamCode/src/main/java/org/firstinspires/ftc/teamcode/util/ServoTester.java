package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Map;
import java.util.stream.Stream;

@TeleOp(name = "Servo Tester", group = "Test")
public class ServoTester extends OpMode {
    private static final double POS_DELTA = 0.02;

    private Servo servo;
    private Servo servo2;
    private Servo servo3;
    private final InputColumnResponder input = new InputColumnResponderImpl();
    private Selector servoSelector;

    @Override
    public void init() {
        Stream<String> lst = hardwareMap.servo.entrySet().stream().map(Map.Entry::getKey);

        servoSelector = new Selector(lst, new String[]{"arm and wrist"});
        input.register(() -> gamepad1.x, servoSelector::selectNext);
    }

    @Override
    public void init_loop() {
        input.update();
        telemetry.addData("Selected", servoSelector.selected());
        telemetry.addLine("Press X to select next");
    }

    private void setArm(double position) {
        servo.setPosition(position);
        servo2.setPosition(1.025 - position);
    }

    @Override
    public void start() {
        if (!servoSelector.selected().equalsIgnoreCase("arm and wrist")) {
            servo = hardwareMap.servo.get(servoSelector.selected());
            input.clearRegistry();

            input.register(() -> gamepad1.dpad_up, () -> servo.setDirection(Servo.Direction.FORWARD))
                    .register(() -> gamepad1.dpad_down, () -> servo.setDirection(Servo.Direction.REVERSE))
                    .register(() -> gamepad1.y, () -> servo.setPosition(servo.getPosition() + POS_DELTA))
                    .register(() -> gamepad1.a, () -> servo.setPosition(servo.getPosition() - POS_DELTA));
        } else {
            servo = hardwareMap.servo.get("armLeft");
            servo2 = hardwareMap.servo.get("armRight");
            servo3 = hardwareMap.servo.get("wrist");
            input.clearRegistry();

            input.register(() -> gamepad1.x, () -> servo3.setPosition(servo3.getPosition() + POS_DELTA))
                    .register(() -> gamepad1.b, () -> servo3.setPosition(servo3.getPosition() - POS_DELTA))
                    .register(() -> gamepad1.y, () -> setArm(servo.getPosition() + POS_DELTA))
                    .register(() -> gamepad1.a, () -> setArm(servo.getPosition() - POS_DELTA));
        }

    }

    @Override
    public void loop() {
        telemetry.addLine("Controls")
                .addData("Dpad Up", "Sets direction to FORWARD")
                .addData("Dpad Down", "Sets direction to REVERSE")
                .addData("Y", "Increments position by %.3f", POS_DELTA)
                .addData("A", "Decrements position by %.3f", POS_DELTA);

        if (!servoSelector.selected().equalsIgnoreCase("arm and wrist")) {
            telemetry.addLine("Servo Data")
                    .addData("Direction", servo.getDirection().name())
                    .addData("Position", "%.4f", servo.getPosition());
        } else {
            telemetry.addLine("Arm Left Data")
                    .addData("Direction", servo.getDirection().name())
                    .addData("Position", "%.4f", servo.getPosition());
            telemetry.addLine("Arm Right Data")
                    .addData("Direction", servo2.getDirection().name())
                    .addData("Position", "%.4f", servo2.getPosition());
            telemetry.addLine("Wrist Data")
                    .addData("Direction", servo3.getDirection().name())
                    .addData("Position", "%.4f", servo3.getPosition());
        }


        input.update();
    }
}
