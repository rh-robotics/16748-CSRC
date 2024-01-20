package org.firstinspires.ftc.teamcode.subsystems.stateMachineController;

public class Obstacle {
    // Assuming for now that all obstacles are rectangular.
    double[] walls;
    double[] dialatedWalls;
    final double MAX_ROBOT_RADIUS = 547.32; // mm
    public Obstacle(double leftX, double rightX, double topY, double bottomY) {
        walls = new double[] {leftX, rightX, topY, bottomY};
        dialatedWalls = new double[] {leftX + MAX_ROBOT_RADIUS, rightX + MAX_ROBOT_RADIUS,
                topY + MAX_ROBOT_RADIUS, bottomY + MAX_ROBOT_RADIUS};
    }
}
