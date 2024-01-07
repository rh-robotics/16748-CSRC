package org.firstinspires.ftc.teamcode.auton.stateMachine;

class Context {
    boolean teamPropLineScored = false;
    boolean teamPropBackdropScored = false;
    boolean robotParked = false;
    boolean inScoringPosition = false;
    boolean inIntakePosition = false;
    byte pixelsInControl = 0;

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
}