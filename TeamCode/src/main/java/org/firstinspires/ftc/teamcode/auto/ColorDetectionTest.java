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

@Autonomous(name = "Test Color Detection", group = "Test")
public class ColorDetectionTest extends LinearOpMode {

    private Bot bot;
    private double cycleTime = 1;
    private Scalar HSV;

    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx gp1 = new GamepadEx(gamepad1);
        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        ColorDetectionPipeline colorPipeline = null;


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
        camera.setPipeline(new ColorDetectionPipeline(telemetry));


        bot = Bot.getInstance(this);
        bot.initializeImus();
        boolean choice = false;
        int highOrLow = 0;
        while (!isStarted()) {
            gp1.readButtons();
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                if (choice) {
                    ColorDetectionTryPipeline colorDetectionPipeline = new ColorDetectionTryPipeline(telemetry);
                    camera.setPipeline(colorDetectionPipeline);
                } else {
                    colorPipeline = new ColorDetectionPipeline(telemetry);
                    camera.setPipeline(colorPipeline);
                }
                choice = !choice;
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                highOrLow = (highOrLow ==0) ? 1 : 0;
            }

            //dpad up down does H high
            //left right does s high
            //bumper/trigger does v
            try {
                if (gp1.wasJustPressed(GamepadKeys.Button.START)) {
                    switch (ColorDetectionPipeline.alliance) {
                        case 0:
                        case 1:
                            colorPipeline.setAlliance(2);
                            break;
                        case 2:
                            colorPipeline.setAlliance(1);
                            break;
                    }
                }

                int inc = 1;
                if (highOrLow == 1) {
                    HSV = colorPipeline.currentHighHSV;
                    if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                        colorPipeline.currentHighHSV.set(new double[]{HSV.val[0]+ inc, HSV.val[1], HSV.val[2]});
                    } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                        colorPipeline.currentHighHSV.set(new double[]{HSV.val[0]- inc, HSV.val[1], HSV.val[2]});
                    }
                    if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                        colorPipeline.currentHighHSV.set(new double[]{HSV.val[0], HSV.val[1]+ inc, HSV.val[2]});
                    } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                        colorPipeline.currentHighHSV.set(new double[]{HSV.val[0], HSV.val[1]- inc, HSV.val[2]});
                    }
                    if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        colorPipeline.currentHighHSV.set(new double[]{HSV.val[0], HSV.val[1], HSV.val[2]+ inc});
                    } else if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                        colorPipeline.currentHighHSV.set(new double[]{HSV.val[0], HSV.val[1], HSV.val[2]- inc});
                    }
                } else {
                    HSV = colorPipeline.currentLowHSV;
                    if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                        colorPipeline.currentLowHSV.set(new double[]{HSV.val[0]+ inc, HSV.val[1], HSV.val[2]});
                    } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                        colorPipeline.currentLowHSV.set(new double[]{HSV.val[0]- inc, HSV.val[1], HSV.val[2]});
                    }
                    if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                        colorPipeline.currentLowHSV.set(new double[]{HSV.val[0], HSV.val[1]+ inc, HSV.val[2]});
                    } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                        colorPipeline.currentLowHSV.set(new double[]{HSV.val[0], HSV.val[1]- inc, HSV.val[2]});
                    }
                    if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        colorPipeline.currentLowHSV.set(new double[]{HSV.val[0], HSV.val[1], HSV.val[2]+ inc});
                    } else if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                        colorPipeline.currentLowHSV.set(new double[]{HSV.val[0], HSV.val[1], HSV.val[2]- inc});
                    }
                }

            } catch(Exception e) {
                telemetry.addLine("Cannot change HSV due to test pipeline not configured ...");
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
                telemetry.addData("H value (Up - Dpad UP, Down - Dpad DOWN)", HSV.val[0]);
                telemetry.addData("S value (Up - Dpad RIGHT, Down - Dpad LEFT)", HSV.val[1]);
                telemetry.addData("V value (Up - RIGHT Bumper, Down - LEFT Bumper)", HSV.val[2]);

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