package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipelines.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.pipelines.ColorDetectionTryPipeline;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;

@Autonomous(name = "Test Color Detection", group = "Test")
public class ColorDetectionTest extends LinearOpMode {

    private Bot bot;
    private double cycleTime = 1;
    private Scalar HSV = ColorDetectionPipeline.currentHighHSV;

    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx gp1 = new GamepadEx(gamepad1);
        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        ColorDetectionPipeline colorPipeline = new ColorDetectionPipeline(telemetry);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error code:", errorCode);
            }
        });
        camera.setPipeline(colorPipeline);


        bot = Bot.getInstance(this);
        bot.initializeImus();
        boolean choice = false;
        int highOrLow = 0;
        while (!isStarted()) {
            gp1.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                highOrLow = (highOrLow ==0) ? 1 : 0;
            }

            //dpad up down does H high
            //left right does s high
            //bumper/trigger does v
                if (gp1.wasJustPressed(GamepadKeys.Button.START)) {
                    switch (ColorDetectionPipeline.alliance) {
                        case 1:
                            colorPipeline.setAlliance(2);
                            ColorDetectionPipeline.count = 0;
                            break;
                        default:
                            colorPipeline.setAlliance(1);
                            ColorDetectionPipeline.count = 0;
                            break;
                    }
                }

                int inc = 1;

                if (highOrLow == 0) {
                    HSV = ColorDetectionPipeline.currentHighHSV;
                    if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                        ColorDetectionPipeline.currentHighHSV.set(new double[]{HSV.val[0]+ inc, HSV.val[1], HSV.val[2]});
                    } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                        ColorDetectionPipeline.currentHighHSV.set(new double[]{HSV.val[0]- inc, HSV.val[1], HSV.val[2]});
                    }
                    if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                        ColorDetectionPipeline.currentHighHSV.set(new double[]{HSV.val[0], HSV.val[1]+ inc, HSV.val[2]});
                    } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                        ColorDetectionPipeline.currentHighHSV.set(new double[]{HSV.val[0], HSV.val[1]- inc, HSV.val[2]});
                    }
                    if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        ColorDetectionPipeline.currentHighHSV.set(new double[]{HSV.val[0], HSV.val[1], HSV.val[2]+ inc});
                    } else if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                        ColorDetectionPipeline.currentHighHSV.set(new double[]{HSV.val[0], HSV.val[1], HSV.val[2]- inc});
                    }
                } else {
                    HSV = ColorDetectionPipeline.currentLowHSV;
                    if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                        ColorDetectionPipeline.currentLowHSV.set(new double[]{HSV.val[0]+ inc, HSV.val[1], HSV.val[2]});
                    } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                        ColorDetectionPipeline.currentLowHSV.set(new double[]{HSV.val[0]- inc, HSV.val[1], HSV.val[2]});
                    }
                    if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                        ColorDetectionPipeline.currentLowHSV.set(new double[]{HSV.val[0], HSV.val[1]+ inc, HSV.val[2]});
                    } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                        ColorDetectionPipeline.currentLowHSV.set(new double[]{HSV.val[0], HSV.val[1]- inc, HSV.val[2]});
                    }
                    if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        ColorDetectionPipeline.currentLowHSV.set(new double[]{HSV.val[0], HSV.val[1], HSV.val[2]+ inc});
                    } else if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                        ColorDetectionPipeline.currentLowHSV.set(new double[]{HSV.val[0], HSV.val[1], HSV.val[2]- inc});
                    }
                }



            telemetry.addLine("Press A to toggle pipeline");
            telemetry.addLine("");
            if (choice) {
                telemetry.addLine("Current Pipeline: Vig's ColorDetectionPipeline");
                telemetry.addData("Spikemark Status", ColorDetectionTryPipeline.spikeMark);
                telemetry.addData("width", ColorDetectionTryPipeline.width);

            } else {
                telemetry.addLine("Current Pipeline: ColorDetectionPipeline");
                telemetry.addData("Spikemark Status", ColorDetectionPipeline.spikeMark);
                telemetry.addData("width", ColorDetectionPipeline.width);
                telemetry.addData("Alliance(Toggle with START)", ColorDetectionPipeline.alliance);
                telemetry.addData("High or Low(Toggle with BACK)", (highOrLow == 0) ? "HIGH" : "LOW");
                telemetry.addData("HSV RAW", Arrays.toString(HSV.val));
                telemetry.addData("H value 180 (Up - Dpad UP, Down - Dpad DOWN)", HSV.val[0]);
                telemetry.addData("S value 255 (Up - Dpad LEFT, Down - Dpad RIGHT)", HSV.val[1]);
                telemetry.addData("V value 255 (Up - RIGHT Bumper, Down - LEFT Bumper)", HSV.val[2]);

            }
            telemetry.update();
        }

        try {
            camera.stopStreaming();
            camera.closeCameraDevice();
        } catch (OpenCvCameraException e) { }
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (choice) {
                telemetry.addLine("Current Pipeline: Vig's ColorDetectionPipeline");
                telemetry.addData("Spikemark Status", ColorDetectionTryPipeline.spikeMark);
                telemetry.addData("width", ColorDetectionTryPipeline.width);
                telemetry.addData("cycle", time - cycleTime);
                cycleTime = time;
            } else {
                telemetry.addLine("Current Pipeline: ColorDetectionPipeline");
                telemetry.addData("Spikemark Status", ColorDetectionPipeline.spikeMark);
                telemetry.addData("width", ColorDetectionPipeline.width);
                telemetry.addData("cycle", time - cycleTime);
                cycleTime = time;

            }
            telemetry.update();
        }







    }
}