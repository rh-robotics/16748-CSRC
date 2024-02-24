package org.firstinspires.ftc.teamcode.subsystems.stateMachineController;

public class Context {
    boolean teamPropLineScored = false;
    boolean teamPropBackdropScored = false;
    boolean robotParked = false;
    boolean inScoringPosition = false;
    boolean inIntakePosition = false;

    // Blue Back, Blue Front, Right Back, Right Front
    int startPos = 0;
    double[][] startLocations = new double[][] {new double[]{50, 450, 270}, new double[]{50, 250, 270},
            new double[]{550, 50, 90}, new double[]{550, 250, 90}};

    double[][] crossingLocations = new double[][] {new double[]{150, 350}, new double[]{150, 350},
            new double[]{450, 350}, new double[]{450, 350}};

    byte pixelsInControl = 0;
    double[] location = new double[] {-1.0, -1.0, 0};

    public Context(int startPos) {
        this.startPos = startPos;
        setLocation(startLocations[startPos][0], startLocations[startPos][1], startLocations[startPos][2]);
    }

    public void setStartPos(int startPos) {
        this.startPos = startPos;
    }

    public void setLocation(double x, double y, double direction) {
        this.location[0] = x;
        this.location[1] = y;
        this.location[2] = direction;
    }

    public void setLocation(double x, double y) {
        this.location[0] = x;
        this.location[1] = y;
    }

    public void setTeamPropLineScored(boolean teamPropLineScored) {
        this.teamPropLineScored = teamPropLineScored;
    }

    public void setTeamPropBackdropScored(boolean teamPropBackdropScored) {
        this.teamPropBackdropScored = teamPropBackdropScored;
    }

    public void setRobotParked(boolean robotParked) {
        this.robotParked = robotParked;
    }

    public void setInScoringPosition(boolean inScoringPosition) {
        this.inScoringPosition = inScoringPosition;
    }

    public void setInIntakePosition(boolean inIntakePosition) {
        this.inIntakePosition = inIntakePosition;
    }

    public void setPixelsInControl(byte pixelsInControl) {
        this.pixelsInControl = pixelsInControl;
    }

    public boolean getTeamPropLineScored() {
        return this.teamPropLineScored;
    }

    public boolean getTeamPropBackdropScored() {
        return this.teamPropBackdropScored;
    }

    public boolean getRobotParked() {
        return this.robotParked;
    }

    public boolean getInScoringPosition() {
        return this.inScoringPosition;
    }

    public boolean getInIntakePosition(){
        return this.inScoringPosition;
    }

    public byte getPixelsInControl() {
        return this.pixelsInControl;
    }

    public double[] getLocation() {
        return this.location;
    }

    public double getX() {
        return this.location[0];
    }

    public double getY() {
        return this.location[0];
    }

    public double getDirection() {
        return this.location[2];
    }

    public double[] getStartPos() {
        return this.startLocations[startPos];
    }

    public double[] getCrossingPos() {
        return this.crossingLocations[startPos];
    }

    public void setDirection(double direction) {
        this.location[2] = direction % 360;
    }
}