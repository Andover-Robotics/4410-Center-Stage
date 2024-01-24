package org.firstinspires.ftc.teamcode.auto;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.pipelines.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.pipelines.ColorDetectionPipeline2;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Test CrCb Color", group = "competition")
public class ColorCrCbDetectionTest extends LinearOpMode {
    public static int alliance = 0;
    public static int spike = 0; // 0 - Center, Left, 1 - Center and Left
    public static final double delta = 0.2;
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx gp1 = new GamepadEx(gamepad1);
        String[] whichSpike = new String[]{"Center, Left", "Center and Left"};
        String[] whichAlliance = new String[]{"None", "Red", "Blue"};

        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        ColorDetectionPipeline2 colorPipeline = new ColorDetectionPipeline2(telemetry);

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

        while (!isStarted()) {
            gp1.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.START)) {
                alliance = (alliance == 0) ? 1 : (alliance == 1) ? 2 : 1;
            }

            colorPipeline.setAlliance(alliance);

            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                spike = (spike == 1) ? 0 : 1;
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                switch(spike) {
                    case 0:
                        colorPipeline.avgCenter+=delta;
                        break;
                    case 1:
                        colorPipeline.avgCenter+=delta;
                        colorPipeline.avgLeft+=delta;
                        break;
                }
            } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                switch(spike) {
                    case 0:
                        colorPipeline.avgCenter-=delta;
                        break;
                    case 1:
                        colorPipeline.avgCenter-=delta;
                        colorPipeline.avgLeft-=delta;
                        break;
                }
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                switch(spike) {
                    case 0:
                        colorPipeline.avgLeft+=delta;
                        break;
                    case 1:
                        colorPipeline.avgCenter+=delta;
                        colorPipeline.avgLeft+=delta;
                        break;
                }
            } else if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                switch(spike) {
                    case 0:
                        colorPipeline.avgLeft-=delta;
                        break;
                    case 1:
                        colorPipeline.avgCenter-=delta;
                        colorPipeline.avgLeft-=delta;
                        break;
                }
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                colorPipeline.minimumAvg+=delta;
            } else if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                colorPipeline.minimumAvg -= delta;
            }

            telemetry.addData("Alliance", whichAlliance[alliance]);
            telemetry.addData("Spike Mark", whichSpike[spike]);
            telemetry.addData("Center Average (UP - DPAD UP, DOWN - DPAD DOWN)", colorPipeline.getAvgCenter());
            if (spike == 1) {
                telemetry.addData("Left Average", colorPipeline.getAvgLeft());
            } else {
                telemetry.addData("Left Average (UP = DPAD RIGHT, DOWN - DPAD LEFT", colorPipeline.getAvgLeft());
            }
            telemetry.addData("Minimum Average (UP - Y, DOWN - A)",colorPipeline.minimumAvg);
            telemetry.update();
        }
        try {
            camera.stopStreaming();
            camera.closeCameraDevice();
        } catch (OpenCvCameraException e) { }

    }
}
