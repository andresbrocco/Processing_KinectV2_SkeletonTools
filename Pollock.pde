/**
 * Pollock: Trigger Affordance that quantifies the "throwing" movement of each hand.
 */
public class Pollock{
  String whichHand;
  private Joint handJoint;
  private Joint shoulderJoint;
  private Joint headJoint;
  private Joint spineShoulderJoint;
  private PVector shoulderToHandPosition;
  private PVector shoulderToHandDirection; // Normalized shoulderToHandPosition
  private PVector shoulderToHandVelocity = new PVector(0, 0, 0);
  private float shoulderToHandSpeed = 0;
  private final float speedThreshold = 0.75;
  private boolean wasAboveSpeedThreshold = false;
  private boolean isAboveSpeedThreshold = false;
  private int activationMillis;
  private PVector headToHand;
  private PVector activationDirection; // vector from head to hand normalized.
  private final int numberOfAltitudes = 3;
  private final int numberOfAzimuths = 4;
  private final int forwardIsBetweenPossibleDirections = 0; // 1:forward is between possible directions. 0:forward is a possible direction.
  private Matrix possibleDirections = new Matrix(numberOfAltitudes*numberOfAzimuths, 3);
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
    this.buildPossibleDirections();
  }
  
  private void buildPossibleDirections(){
    int directionIndex = 0;
    for(int altitudeIndex=0; altitudeIndex<this.numberOfAltitudes; altitudeIndex++){
      for(int azimuthIndex=0; azimuthIndex<this.numberOfAzimuths; azimuthIndex++){
        this.possibleDirections.set(directionIndex, 0, sin((altitudeIndex+1)*PI/(this.numberOfAltitudes+1))*sin(azimuthIndex*TWO_PI/this.numberOfAzimuths+forwardIsBetweenPossibleDirections*PI/this.numberOfAzimuths));
        this.possibleDirections.set(directionIndex, 1, cos((altitudeIndex+1)*PI/(this.numberOfAltitudes+1)));
        this.possibleDirections.set(directionIndex, 2, sin((altitudeIndex+1)*PI/(this.numberOfAltitudes+1))*cos(azimuthIndex*TWO_PI/this.numberOfAzimuths+forwardIsBetweenPossibleDirections*PI/this.numberOfAzimuths));
        //println("direction "+directionIndex+": "+this.possibleDirections.get(directionIndex, 0)+" "+ this.possibleDirections.get(directionIndex, 1)+" "+ this.possibleDirections.get(directionIndex, 2));
        directionIndex++;
      }
    }
  }
  
  private void update(){
    this.shoulderToHandPosition = PVector.sub(this.handJoint.estimatedPosition, this.shoulderJoint.estimatedPosition);
    this.shoulderToHandDirection = PVector.div(this.shoulderToHandPosition, this.shoulderToHandPosition.mag());
    this.shoulderToHandVelocity = PVector.sub(this.handJoint.estimatedVelocity, this.shoulderJoint.estimatedVelocity);
    this.shoulderToHandSpeed = PVector.dot(this.shoulderToHandVelocity, this.shoulderToHandDirection);
    this.isAboveSpeedThreshold = this.shoulderToHandSpeed > this.speedThreshold;
    
    if(this.isAboveSpeedThreshold){
      this.headToHand = PVector.sub(this.handJoint.estimatedPosition, this.headJoint.estimatedPosition);
    } else if(this.wasAboveSpeedThreshold){ // Pollock is activated here
      this.activationMillis = millis();
      this.findDirection();
    }
    this.wasAboveSpeedThreshold = this.isAboveSpeedThreshold;
  }
  
  private void findDirection(){
    this.activationDirection = PVector.div(this.headToHand, headToHand.mag());
    // CODE HERE
  }
  
  private void draw(){
    this.drawTriggerVector();
    this.drawPossibleDirectionsInTheBody();
    this.drawPossibleDirectionsInTheOrigin();
    
    if(millis()-this.activationMillis < 1000){
      this.drawHeadToHandVector();
      this.drawHeadToHandVectorInTheOrigin();
      this.drawActivationDirectionInTheOrigin();
    }
  }
  
  private void drawTriggerVector(){
    pushMatrix();
    strokeWeight(5);
    stroke(0, 0, 0, max(0, min(255, map(this.shoulderToHandSpeed, 0, 2, 0, 255))));
    translate(reScaleX(this.shoulderJoint.estimatedPosition.x, "pollock.draw"),
              reScaleY(this.shoulderJoint.estimatedPosition.y, "pollock.draw"),
              reScaleZ(this.shoulderJoint.estimatedPosition.z, "pollock.draw"));
    PVector shoulderToHand = PVector.sub(this.handJoint.estimatedPosition, this.shoulderJoint.estimatedPosition);
    line(0, 0, 0, reScaleX(shoulderToHand.x, "pollock.draw"), reScaleY(shoulderToHand.y, "pollock.draw"), reScaleZ(shoulderToHand.z, "pollock.draw"));
    popMatrix();
  }
  
  private void drawHeadToHandVector(){
    pushMatrix();
    strokeWeight(5);
    stroke(0, 0, 0, 128);
    translate(reScaleX(this.headJoint.estimatedPosition.x, "pollock.draw"),
              reScaleY(this.headJoint.estimatedPosition.y, "pollock.draw"),
              reScaleZ(this.headJoint.estimatedPosition.z, "pollock.draw")); 
    line(0, 0, 0, reScaleX(this.headToHand.x, "pollock.draw"), reScaleY(this.headToHand.y, "pollock.draw"), reScaleZ(this.headToHand.z, "pollock.draw"));
    popMatrix();
  }
  
  private void drawHeadToHandVectorInTheOrigin(){
    pushMatrix();
    strokeWeight(5);
    stroke(0, 0, 0, 128);
    line(0, 0, 0, reScaleX(this.headToHand.x, "pollock.draw"), reScaleY(this.headToHand.y, "pollock.draw"), reScaleZ(this.headToHand.z, "pollock.draw"));
    popMatrix();
  }
  
  private void drawActivationDirectionInTheOrigin(){
    pushMatrix();
    strokeWeight(5);
    stroke(0, 0, 0, 128);
    line(0, 0, 0, reScaleX(this.activationDirection.x, "pollock.draw"), reScaleY(this.activationDirection.y, "pollock.draw"), reScaleZ(this.activationDirection.z, "pollock.draw"));
    popMatrix();
  }
  
  private void drawPossibleDirectionsInTheOrigin(){
    int directionIndex = 0;
    for(int altitudeIndex=0; altitudeIndex<this.numberOfAltitudes; altitudeIndex++){
      for(int azimuthIndex=0; azimuthIndex<this.numberOfAzimuths; azimuthIndex++){
        pushMatrix();
        strokeWeight(2);
        stroke(100, 0, 200, 25+230*directionIndex/(this.numberOfAltitudes*this.numberOfAzimuths));
        PVector directionToDraw = new PVector((float) this.possibleDirections.get(directionIndex, 0), (float) this.possibleDirections.get(directionIndex, 1), (float) this.possibleDirections.get(directionIndex, 2));
        line(0, 0, 0, reScaleX(directionToDraw.x, "pollock.draw"),
                      reScaleY(directionToDraw.y, "pollock.draw"),
                      reScaleZ(directionToDraw.z, "pollock.draw"));
        directionIndex++;
        popMatrix();
      }
    }
  }
  
  private void drawPossibleDirectionsInTheBody(){
    int directionIndex = 0;
    for(int altitudeIndex=0; altitudeIndex<this.numberOfAltitudes; altitudeIndex++){
      for(int azimuthIndex=0; azimuthIndex<this.numberOfAzimuths; azimuthIndex++){
        pushMatrix();
        strokeWeight(2);
        stroke(100, 0, 200, 25+230*directionIndex/(this.numberOfAltitudes*this.numberOfAzimuths));
        translate(reScaleX(this.spineShoulderJoint.estimatedPosition.x, "pollock.draw"),
                  reScaleY(this.spineShoulderJoint.estimatedPosition.y, "pollock.draw"),
                  reScaleZ(this.spineShoulderJoint.estimatedPosition.z, "pollock.draw"));
        PVector directionToDraw = new PVector((float) this.possibleDirections.get(directionIndex, 0), (float) this.possibleDirections.get(directionIndex, 1), (float) this.possibleDirections.get(directionIndex, 2));
        Quaternion directionToDrawQuaternion = new Quaternion(0, directionToDraw);
        PVector relativeDirectionToDraw = qMult(qConjugate(this.spineShoulderJoint.estimatedOrientation), qMult(directionToDrawQuaternion, this.spineShoulderJoint.estimatedOrientation)).vector;
        //PVector relativeDirectionToDraw = new PVector(PVector.dot(directionToDraw, this.spineShoulderJoint.estimatedDirectionX), 
        //                                              PVector.dot(directionToDraw, this.spineShoulderJoint.estimatedDirectionY), 
        //                                              PVector.dot(directionToDraw, this.spineShoulderJoint.estimatedDirectionZ));
        line(0, 0, 0, reScaleX(-relativeDirectionToDraw.x, "pollock.draw"),
                      reScaleY(relativeDirectionToDraw.y, "pollock.draw"),
                      reScaleZ(relativeDirectionToDraw.z, "pollock.draw"));
        directionIndex++;
        popMatrix();
      }
    }
  }
}
