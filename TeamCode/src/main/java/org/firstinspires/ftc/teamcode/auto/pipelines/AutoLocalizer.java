package org.firstinspires.ftc.teamcode.auto.pipelines;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.auto.TestAutonomous;
import org.firstinspires.ftc.teamcode.auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.auto.trajectorysequence.sequencesegment.SequenceSegment;

import java.util.ArrayList;
import java.util.Arrays;

public class AutoLocalizer {
    private TrajectorySequence[] paths = new TrajectorySequence[10];
    private SampleMecanumDrive drive;
    private int follower = -1;
    private boolean finished = true;

    private int index;

    public AutoLocalizer(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    private boolean findOpenIndex() {
        for (int i = 0; i< paths.length; i++) {
            if (paths[i] == null) {
                index = i;
                return true;
            }
        }
        return false;
    }

    private int checkNoDuplicates(TrajectorySequence seq) {
        int[] compares = new int[10];
        int in = 0;
        for(int i = 0; i< paths.length; i++) {
            if (paths[i].size() == seq.size()) {
                compares[in] = i;
                in++;
            }
        }
        for (int i = 0; i< seq.size(); i++) {
            for(int j = 0; j< compares.length; j++) {
                for(int k = 0; k< paths[compares[j]].size(); k++) {
                    if (paths[compares[j]].get(k).getStartPose().epsilonEquals(seq.get(i).getStartPose())) {
                        return compares[j];
                    }
                    break;
                }
            }
        }
        return -1;
    }

    public boolean addSequence(TrajectorySequence seq) {
        if (!findOpenIndex()) {
            return false;
        }
        if (checkNoDuplicates(seq) != -1) {
            return false;
        }
        paths[index] = seq;
        return true;
    }

    public int getSequenceIndex(TrajectorySequence seq) {
        return checkNoDuplicates(seq);
    }

    public void periodic() {
        finished = !drive.isBusy();
    }

    public boolean runSequence(int in) {
        if (in < 0 || in > paths.length || paths[in] == null) {
            return false;
        }
        follower = in;
        drive.followTrajectorySequenceAsync(paths[in]);

        return drive.isBusy();
    }

    public TrajectorySequence followLastSequence(boolean run) {
        if (finished) {
            if (run) {
                runSequence(follower);
            }
            return paths[follower];
        } else {
            drive.trajectorySequenceRunner.currentTrajectorySequence.end();
            periodic();
            return followLastSequence(run);
        }
    }
}
