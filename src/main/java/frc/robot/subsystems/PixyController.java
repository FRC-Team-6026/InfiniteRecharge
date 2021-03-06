package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2Line;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.Pixy2Line.Vector;

public class PixyController extends SubsystemBase{
    //Pixy Width in CCC mode = 316
    //Pixy Height in CCC mode = 208
    //Pixy Width in line mode = 78
    //Pixy Height in line mode = 51
    private final Pixy2 _pixy = Pixy2.createInstance(LinkType.SPI);
    private final PIDController _ballPid = new PIDController(0.001, 0, 0);
    private final PIDController _targetPid = new PIDController(0.02, 0, 0);
    private double _previousTargetRotation = 0;

    public void init(){
        _pixy.init();
        _ballPid.setSetpoint(158);
        _ballPid.setTolerance(2);
        SendableRegistry.setName(_ballPid, "Pixy Controller", "Ball PID");
        _targetPid.setSetpoint(39);
        _targetPid.setTolerance(2);
        SendableRegistry.setName(_targetPid, "Pixy Controller", "Target PID");
    }

    /**
     * Calculates the rotation necessary to track the yellow
     * game ball.
     * @return A turn value between -1 and 1.
     */
    public double trackBall() {
        var biggestBlock = getBiggestBlock();
        if (biggestBlock == null){
            return 0;
        }
        biggestBlock.getAngle();
        var midpointX = biggestBlock.getX();
        SmartDashboard.putNumber("Pixy X midpoint", midpointX);
        //negative rotation inceases the measured x value, so the PID
        //response has to be inverted
        var rotation = -_ballPid.calculate(midpointX);
        rotation = Math.min(rotation, 1);
        rotation = Math.max(rotation, -1);
        return rotation;
    }

    /**
     * Calculates the rotation necessary to track the upper
     * target
     * @return
     */
    public double trackTarget() {
        var leftAndRightVector = getLeftAndRightVector();
        if (leftAndRightVector == null){
            _previousTargetRotation = _previousTargetRotation * .5;
            return _previousTargetRotation;
        }
        var left = leftAndRightVector[0];
        var right = leftAndRightVector[1];
        var midpointX = left.getX0() + ((right.getX0() - left.getX0()) / 2.0);
        SmartDashboard.putNumber("Pixy Top left X", left.getX0());
        SmartDashboard.putNumber("Pixy Top right X", right.getX0());
        SmartDashboard.putNumber("Pixy X midpoint", midpointX);
        //negative rotation increases the measured x so the PID response
        //has to be inverted
        var rotation = -_targetPid.calculate(midpointX);
        rotation = Math.min(rotation, 1);
        rotation = Math.max(rotation, -1);
        _previousTargetRotation = rotation;
        return rotation;
    }

    public void turnLightOn(int r, int g, int b){
        _pixy.setLamp((byte)1, (byte)1);
        _pixy.setLED(r, g, b);
    }

    public void turnLightOff(){
        _pixy.setLamp((byte)0, (byte)0);
    }

    public void setBrightness(int brightness){
        _pixy.setCameraBrightness(brightness);
    }

    private Block getBiggestBlock() {
		// Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
		// does not wait for new data if none is available,
		// and limits the number of returned blocks to 15, for a slight increase in efficiency
		int blockCount = _pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 15);
		if (blockCount <= 0) {
			return null; // If blocks were not found, stop processing
		}
		ArrayList<Block> blocks = _pixy.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2
		Block largestBlock = null;
		for (Block block : blocks) { // Loops through all blocks and finds the widest one
			if (largestBlock == null) {
				largestBlock = block;
			} else if ((block.getWidth()*block.getHeight()) > (largestBlock.getWidth()*largestBlock.getHeight())) {
				largestBlock = block;
			}
		}
		return largestBlock;
    }
    
    private Vector[] getLeftAndRightVector() {
        var line = _pixy.getLine();
        line.getAllFeatures(Pixy2Line.LINE_VECTOR, false);
        var vectors = line.getVectorCache();
        if (vectors == null){
            return null;
        }
        ArrayList<Vector> leftCandidates = new ArrayList<Vector>();
        ArrayList<Vector> rightCandidates = new ArrayList<Vector>();
        for(var v : vectors){
            var topX = 0;
            var topY = 0;
            var botX = 0;
            var botY = 0;
            //Y increased going down in the image, so a small value is "higher"
            if (v.getY0() < v.getY1()) {
                topX = v.getX0();
                topY = v.getY0();
                botX = v.getX1();
                botY = v.getY1();
            } else {
                topX = v.getX1();
                topY = v.getY1();
                botX = v.getX0();
                botY = v.getY0();
            }
            //The Y diff should always be positive
            var slope = (double)(botY - topY) / (botX - topX);

            var absSlope = Math.abs(slope);
            //check to make sure slope is approximately 50 to 70 degree
            //theoretically it should be 60 degrees.
            if (absSlope <= 1.19 || absSlope >= 2.75) {
                continue;
            }

            if (slope < 0) {
                rightCandidates.add(new Vector(topX,topY,botX,botY, 0, 0));
            } else {
                leftCandidates.add(new Vector(topX,topY,botX,botY, 0, 0));
            }
        }
        for(var left : leftCandidates) {
            for(var right : rightCandidates) {
                var topYDiff = Math.abs(left.getY0() - right.getY0());
                var botYDiff = Math.abs(left.getY1() - right.getY1());
                if (topYDiff > 5 || botYDiff > 5) {
                    continue;
                }
                var leftRightVectors = new Vector[2];
                leftRightVectors[0] = left;
                leftRightVectors[1] = right;
                return leftRightVectors;
            }
        }
        return null;
    }
}