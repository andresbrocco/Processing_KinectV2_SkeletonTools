/**
 * Pollock: Trigger Affordance that quantifies the "throwing" movement of each hand.
 */
public class Pollock{
  String whichHand;
  private Joint handJoint;
  private Joint shoulderJoint;
  private Joint headJoint;
  private Joint spineShoulderJoint;
  private PVector shoulderToHandVelocity = new PVector(0, 0, 0);
  private float shoulderToHandSpeed = 0;
  private final float speedThreshold = 0.75;
  private boolean isAboveSpeedThreshold = false;
  private int activationMillis;
  private PVector headPositionWhenActivated;
  private PVector handPositionWhenActivated;
  private final int numberOfAltitudes = 3;
  private final int numberOfAzimuths = 4;
  private final int forwardIsBetweenPollockDirections = 0; // 1:forward is between pollock directions. 0:forward is a pollock direction.
  private Matrix directionsMatrix = new Matrix(numberOfAltitudes*numberOfAzimuths, 3);
  private int direction = 0;
  
  Pollock(Skeleton skeleton, String whichHand){
    this.whichHand = whichHand;
    if(whichHand == "LEFT"){
      this.handJoint = skeleton.joints[HAND_LEFT];
      this.shoulderJoint = skeleton.joints[SHOULDER_LEFT];
    }else if(whichHand == "RIGHT"){
      this.handJoint = skeleton.joints[HAND_RIGHT];
      this.shoulderJoint = skeleton.joints[SHOULDER_RIGHT];
    }
    this.headJoint = skeleton.joints[HEAD];
    this.spineShoulderJoint = skeleton.joints[SPINE_SHOULDER];
    int directionIndex = 0;
    for(int altitudeIndex=0; altitudeIndex<this.numberOfAltitudes; altitudeIndex++){
      for(int azimuthIndex=0; azimuthIndex<this.numberOfAzimuths; azimuthIndex++){
        this.directionsMatrix.set(directionIndex, 0, sin((altitudeIndex+1)*PI/(this.numberOfAltitudes+1))*sin(azimuthIndex*TWO_PI/this.numberOfAzimuths+forwardIsBetweenPollockDirections*PI/this.numberOfAzimuths));
        this.directionsMatrix.set(directionIndex, 1, cos((altitudeIndex+1)*PI/(this.numberOfAltitudes+1)));
        this.directionsMatrix.set(directionIndex, 2, sin((altitudeIndex+1)*PI/(this.numberOfAltitudes+1))*cos(azimuthIndex*TWO_PI/this.numberOfAzimuths+forwardIsBetweenPollockDirections*PI/this.numberOfAzimuths));
        //println("direction "+directionIndex+": "+this.directionsMatrix.get(directionIndex, 0)+" "+ this.directionsMatrix.get(directionIndex, 1)+" "+ this.directionsMatrix.get(directionIndex, 2));
        directionIndex++;
      }
    }
  }
  
  private void update(){
    this.shoulderToHandVelocity = PVector.sub(this.handJoint.estimatedVelocity, this.shoulderJoint.estimatedVelocity);
    PVector spineShoulderToHandDirection = PVector.sub(this.handJoint.estimatedPosition, this.shoulderJoint.estimatedPosition).normalize();
    this.shoulderToHandSpeed = PVector.dot(this.shoulderToHandVelocity, spineShoulderToHandDirection);
    println("joint " + this.handJoint.id + " Speed: " + this.shoulderToHandSpeed);
    if(this.isAboveSpeedThreshold){
      if(this.shoulderToHandSpeed > this.speedThreshold){
        this.handPositionWhenActivated = this.handJoint.estimatedPosition;
        this.headPositionWhenActivated = this.headJoint.estimatedPosition;
      } else{
        this.isAboveSpeedThreshold = false;
        this.activationMillis = millis();
      }
    } else if(this.shoulderToHandSpeed > this.speedThreshold){
      this.handPositionWhenActivated = this.handJoint.estimatedPosition;
      this.headPositionWhenActivated = this.headJoint.estimatedPosition;
      this.isAboveSpeedThreshold = true;
    }
    this.findDirection();
  }
  
  private void findDirection(){
    
  }
  
  private void draw(){
    pushMatrix();
    strokeWeight(5);
    stroke(0, 0, 0, max(0, min(255, map(this.shoulderToHandSpeed, 0, 2, 0, 255))));
    translate(reScaleX(this.shoulderJoint.estimatedPosition.x, "pollock.draw"),
              reScaleY(this.shoulderJoint.estimatedPosition.y, "pollock.draw"),
              reScaleZ(this.shoulderJoint.estimatedPosition.z, "pollock.draw"));
    PVector spineShoulderToHand = PVector.sub(this.handJoint.estimatedPosition, this.shoulderJoint.estimatedPosition);
    line(0, 0, 0, reScaleX(spineShoulderToHand.x, "pollock.draw"), reScaleY(spineShoulderToHand.y, "pollock.draw"), reScaleZ(spineShoulderToHand.z, "pollock.draw"));
    popMatrix();
    
    if(this.isAboveSpeedThreshold || millis()-this.activationMillis<1000){
      pushMatrix();
      strokeWeight(5);
      stroke(0, 0, 0, 128);
      translate(reScaleX(this.headPositionWhenActivated.x, "pollock.draw"),
                reScaleY(this.headPositionWhenActivated.y, "pollock.draw"),
                reScaleZ(this.headPositionWhenActivated.z, "pollock.draw"));
      PVector headToHand = PVector.sub(this.handPositionWhenActivated, this.headPositionWhenActivated);
      line(0, 0, 0, reScaleX(headToHand.x, "pollock.draw"), reScaleY(headToHand.y, "pollock.draw"), reScaleZ(headToHand.z, "pollock.draw"));
      popMatrix();
    }
    
    int directionIndex = 0;
    for(int altitudeIndex=0; altitudeIndex<this.numberOfAltitudes; altitudeIndex++){
      for(int azimuthIndex=0; azimuthIndex<this.numberOfAzimuths; azimuthIndex++){
        pushMatrix();
        stroke(0, 0, 0, 255*directionIndex/(this.numberOfAltitudes*this.numberOfAzimuths));
        translate(reScaleX(this.spineShoulderJoint.estimatedPosition.x, "pollock.draw"),
                  reScaleY(this.spineShoulderJoint.estimatedPosition.y, "pollock.draw"),
                  reScaleZ(this.spineShoulderJoint.estimatedPosition.z, "pollock.draw"));
        PVector directionToDraw = new PVector((float) this.directionsMatrix.get(directionIndex, 0), (float) this.directionsMatrix.get(directionIndex, 1), (float) this.directionsMatrix.get(directionIndex, 2));
        /*line(0, 0, 0, reScaleX(directionToDraw.x, "pollock.draw"),
                      reScaleY(directionToDraw.y, "pollock.draw"),
                      reScaleZ(directionToDraw.z, "pollock.draw"));*/
        PVector relativeDirectionToDraw = new PVector(PVector.dot(directionToDraw, this.spineShoulderJoint.estimatedDirectionX), 
                                                      PVector.dot(directionToDraw, this.spineShoulderJoint.estimatedDirectionY), 
                                                      PVector.dot(directionToDraw, this.spineShoulderJoint.estimatedDirectionZ));
        line(0, 0, 0, reScaleX(relativeDirectionToDraw.x, "pollock.draw"),
                      reScaleY(relativeDirectionToDraw.y, "pollock.draw"),
                      reScaleZ(relativeDirectionToDraw.z, "pollock.draw"));
        directionIndex++;
        popMatrix();
      }
    }
  }
}






/**
 * Check if the hand triggered the speed threshold. If so, find which direction it is pointing.
 *  0 = below threshold. 
 * The 3D space is divided in 12 directions, where:
 *   The azimuth  is divided in  1=left,  2=front,  3=right,  4=back.
 *   The altitude is divided in 10=up  , 20=front, 30=down , 40=back.
 *   The resulting pollock number is the sum of "azimuth code" with "altitude code" above described.
 *   The possible combinations are:
 *     Altitude   |   Azimuth
 *   --------------------------
 *        Up      |    Left
 *        Up      |    Front
 *        Up      |    Right
 *        Up      |    Back
 *       Down     |    Left
 *       Down     |    Front
 *       Down     |    Right
 *       Down     |    Back
 *       Front    |    Front
 *       Left     |    Left
 *       Back     |    Back
 *       Right    |    Right
 */
