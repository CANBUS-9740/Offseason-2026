package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotMap;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.geometry.Vector3;
import org.opencv.core.Mat;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class FiringDisplay implements NTSendable {

    private static final double FIELD_LENGTH = 16.518;
    private static final double FIELD_HEIGHT = 5;
    private static final double ROBOT_STRUCTURE_LINE_WIDTH = 2;
    private static final Color8Bit ROBOT_STRUCTURE_COLOR = new Color8Bit(Color.kRed);
    private static final double HUB_LENGTH = 0.5;
    private static final double HUB_HEIGHT = RobotMap.HUB_HEIGHT_METERS;
    private static final double HUB_STRUCTURE_LINE_WIDTH = 2;
    private static final Color8Bit HUB_STRUCTURE_COLOR = new Color8Bit(Color.kGreen);
    private static final double TRAJECTORY_STRUCTURE_LINE_WIDTH = 1;
    private static final Color8Bit TRAJECTORY_COLOR = new Color8Bit(Color.kPaleVioletRed);

    private final Mechanism2d mechanism;
    private final MechanismRoot2d robotRoot;
    private final MechanismRoot2d hubBottomLeft;
    private final MechanismRoot2d hubTopRight;
    private final MechanismRoot2d ballRoot;

    private final MechanismRoot2d firingTrajectoryRoot;
    private final List<MechanismLigament2d> firingTrajectoryParts;
    private Vector3 lastBallPosition = null;
    private double lastDx;
    private double lastDy;
    private int nextSegmentIdx = 0;

    public FiringDisplay() {
        mechanism = new Mechanism2d(FIELD_LENGTH, FIELD_HEIGHT);

        robotRoot = mechanism.getRoot("robot", 0, 0);
        robotRoot.append(new MechanismLigament2d("forward", RobotMap.ROBOT_LENGTH_METERS / 2, 0, ROBOT_STRUCTURE_LINE_WIDTH, ROBOT_STRUCTURE_COLOR));
        robotRoot.append(new MechanismLigament2d("backward", RobotMap.ROBOT_LENGTH_METERS / 2, 180, ROBOT_STRUCTURE_LINE_WIDTH, ROBOT_STRUCTURE_COLOR));

        hubBottomLeft = mechanism.getRoot("hubBottomLeft", 0, 0);
        hubBottomLeft.append(new MechanismLigament2d("bottom", HUB_LENGTH, 0, HUB_STRUCTURE_LINE_WIDTH, HUB_STRUCTURE_COLOR));
        hubBottomLeft.append(new MechanismLigament2d("left", HUB_HEIGHT, 90, HUB_STRUCTURE_LINE_WIDTH, HUB_STRUCTURE_COLOR));
        hubTopRight = mechanism.getRoot("hubTopRight", 0, 0);
        hubTopRight.append(new MechanismLigament2d("up", HUB_LENGTH, 180, HUB_STRUCTURE_LINE_WIDTH, HUB_STRUCTURE_COLOR));
        hubTopRight.append(new MechanismLigament2d("right", HUB_HEIGHT, 270, HUB_STRUCTURE_LINE_WIDTH, HUB_STRUCTURE_COLOR));

        ballRoot = mechanism.getRoot("ball", 0, 0);
        ballRoot.append(new MechanismLigament2d("ball", 0.1, 0, 5, new Color8Bit(Color.kBlue)));

        firingTrajectoryRoot = mechanism.getRoot("trajectory", 0, 0);
        firingTrajectoryParts = new ArrayList<>();
    }

    public void setRobotPosition(Pose2d pose) {
        robotRoot.setPosition(pose.getX(), 0);
    }

    public void setHubPosition(Pose2d pose) {
        hubBottomLeft.setPosition(pose.getX() - HUB_LENGTH / 2, 0);
        hubTopRight.setPosition(pose.getX() + HUB_LENGTH / 2, HUB_HEIGHT);
    }

    public void setBallPosition(Vector3 position) {
        ballRoot.setPosition(position.x, position.z);
        updateTrajectory(position);
    }

    public void resetTrajectory() {
        firingTrajectoryParts.forEach((lig)-> {
            lig.setLength(0);
            lig.close();
        });
        firingTrajectoryParts.clear();
        clearObjects(firingTrajectoryRoot);
        nextSegmentIdx = 0;
        lastBallPosition = null;
        lastDx = 0;
        lastDy = 0;
    }

    public void updateTrajectory(Vector3 position) {
        if (lastBallPosition == null) {
            firingTrajectoryRoot.setPosition(position.x, position.z);
            lastBallPosition = new Vector3(position);
            return;
        }

        double dx = position.x - lastBallPosition.x;
        double dy = position.z - lastBallPosition.z;
        if (Math.abs(dx) < 0.5 && Math.abs(dy) < 0.5) {
            return;
        }

        double length = Math.hypot(dx, dy);
        double angle = Math.toDegrees(Math.atan2(dy, dx));

        MechanismLigament2d ligament;
        if (nextSegmentIdx == 0) {
            ligament = new MechanismLigament2d(
                    String.valueOf(nextSegmentIdx),
                    length,
                    angle,
                    TRAJECTORY_STRUCTURE_LINE_WIDTH,
                    TRAJECTORY_COLOR);
            firingTrajectoryRoot.append(ligament);
        } else {
            MechanismLigament2d previousLigament = firingTrajectoryParts.get(nextSegmentIdx - 1);
            double correctedAngle = translateAngle(angle - Math.toDegrees(Math.atan2(lastDy, lastDx)));
            ligament = new MechanismLigament2d(
                    String.valueOf(nextSegmentIdx),
                    length,
                    correctedAngle,
                    TRAJECTORY_STRUCTURE_LINE_WIDTH,
                    TRAJECTORY_COLOR);
            previousLigament.append(ligament);
        }
        firingTrajectoryParts.add(ligament);

        nextSegmentIdx++;
        lastBallPosition = new Vector3(position);
        lastDx = dx;
        lastDy = dy;
    }

    private double translateAngle(double angle) {
        angle %= 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    private void clearObjects(Object object) {
        try {
            Field field = MechanismObject2d.class.getDeclaredField("m_objects");
            field.setAccessible(true);
            Object value = field.get(object);
            ((Map) value).clear();
        } catch (IllegalAccessException | NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void initSendable(NTSendableBuilder ntSendableBuilder) {
        mechanism.initSendable(ntSendableBuilder);
    }
}
