package org.firstinspires.ftc.teamcode.subsystems.stateMachineController;

public class Context {
    boolean teamPropLineScored = false;
    boolean teamPropBackdropScored = false;
    boolean robotParked = false;
    boolean inScoringPosition = false;
    boolean inIntakePosition = false;
    byte pixelsInControl = 0;
    double[] location = new double[] {-1.0, -1.0, 0};
    Obstacle[] obstacles = new Obstacle[] {};

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

    public void setDirection(double direction) {
        this.location[2] = direction % 360;
    }
}