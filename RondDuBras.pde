/**
 * RondDuBras: Trigger Affordance that quantifies the "round" movement of each hand.
 * TODO: sair apagando tudo do pollock que não vai precisar.
 */
public class RondDuBras{
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
  private PVector activationDirectionGlobal; // vector from head to hand normalized.
  private PVector activationDirectionRelativeToBody; // vector from head to hand normalized, relative to shoulder coordinate system.
  private final int numberOfAltitudes = 3; // 3
  private final int numberOfAzimuths = 4; // 4
  private final int forwardIsBetweenPossibleDirections = 0; // 1:forward is between possible directions. 0:forward is a possible direction.
  private Matrix possibleDirections = new Matrix(numberOfAltitudes*numberOfAzimuths, 3);
  private Matrix projectionInEachPossibleDirectionVector = new Matrix(numberOfAltitudes*numberOfAzimuths, 1);
  private int activationDirectionIndex = 0;
  
  RondDuBras(Skeleton skeleton, String whichHand){
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
  
  /**
   * Build a Matrix containing the vectors for all possible directions of the pollock, based on the chosen number of azimuths and altitudes.
   */
  private void buildPossibleDirections(){
    int directionIndex = 0;
    for(int altitudeIndex=0; altitudeIndex<this.numberOfAltitudes; altitudeIndex++){
      for(int azimuthIndex=0; azimuthIndex<this.numberOfAzimuths; azimuthIndex++){
        this.possibleDirections.set(directionIndex, 0, sin((altitudeIndex+1)*PI/(this.numberOfAltitudes+1))*sin(azimuthIndex*TWO_PI/this.numberOfAzimuths+forwardIsBetweenPossibleDirections*PI/this.numberOfAzimuths));
        this.possibleDirections.set(directionIndex, 1, cos((altitudeIndex+1)*PI/(this.numberOfAltitudes+1)));
        this.possibleDirections.set(directionIndex, 2, sin((altitudeIndex+1)*PI/(this.numberOfAltitudes+1))*cos(azimuthIndex*TWO_PI/this.numberOfAzimuths+forwardIsBetweenPossibleDirections*PI/this.numberOfAzimuths));
        directionIndex++;
      }
    }
  }
  
  /**
   * Updates the pollock calculations.
   */
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
  
  /**
   * Find the possible direction that has the largest dot product with the activation direction.   
   */
  private void findDirection(){
    this.activationDirectionGlobal = PVector.div(this.headToHand, headToHand.mag());
    this.activationDirectionRelativeToBody = new PVector(PVector.dot(this.activationDirectionGlobal, this.spineShoulderJoint.estimatedDirectionX), 
                                                         PVector.dot(this.activationDirectionGlobal, this.spineShoulderJoint.estimatedDirectionY), 
                                                         PVector.dot(this.activationDirectionGlobal, this.spineShoulderJoint.estimatedDirectionZ));
    Matrix activationDirectionRelativeToBodyVector = new Matrix(new double[] {this.activationDirectionRelativeToBody.x, this.activationDirectionRelativeToBody.y, this.activationDirectionRelativeToBody.z}, 3);
    this.projectionInEachPossibleDirectionVector = this.possibleDirections.times(activationDirectionRelativeToBodyVector);
    this.activationDirectionIndex = 0;
    double max = this.projectionInEachPossibleDirectionVector.get(0, 0);
    for(int possibleDirection=1; possibleDirection<this.projectionInEachPossibleDirectionVector.getRowDimension(); possibleDirection++){
      if(this.projectionInEachPossibleDirectionVector.get(possibleDirection, 0) > max){
        this.activationDirectionIndex = possibleDirection;
        max = this.projectionInEachPossibleDirectionVector.get(possibleDirection, 0);
      }
    }
    println("activationDirectionIndex: "+this.activationDirectionIndex);
  }
  
  /**
   * Draw the representations of the pollock affordance.
   */
  private void draw(){
    //this.drawTriggerVector();
    //this.drawPossibleDirectionsInTheBody();
    //this.drawPossibleDirectionsInTheOrigin();
    
    if(millis()-this.activationMillis < 1000){
      this.drawHeadToHandVector();
      //this.drawProjectionsInPossibleDirectionsInTheOrigin();
      //this.drawHeadToHandVectorInTheOrigin();
      //this.drawPossibleDirectionsInTheOrigin();
      this.drawActivationDirectionInTheOrigin();
    }
  }
  
  /**
   * Draw the vector from shoulder to arm. Its stroke alpha represents the velocity of the pollock.
   */
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
  
  /**
   * Draw the vector from head to hand, which is the direction of the activation.
   */
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
  
  /**
   * Draw the vector from head to hand in the origin, which is the direction of the activation in global CSys.
   */
  private void drawHeadToHandVectorInTheOrigin(){
    pushMatrix();
    strokeWeight(5);
    stroke(0, 0, 0, 128);
    line(0, 0, 0, reScaleX(this.headToHand.x, "pollock.draw"), reScaleY(this.headToHand.y, "pollock.draw"), reScaleZ(this.headToHand.z, "pollock.draw"));
    popMatrix();
  }
  
  /**
   * Draw the activated possible direction in the origin.
   */
  private void drawActivationDirectionInTheOrigin(){
    pushMatrix();
    strokeWeight(5);
    stroke(100, 0, 200, 255);
    line(0, 0, 0, reScaleX((float)this.possibleDirections.get(this.activationDirectionIndex, 0), "pollock.draw"), reScaleY((float)this.possibleDirections.get(this.activationDirectionIndex, 1), "pollock.draw"), reScaleZ((float)this.possibleDirections.get(this.activationDirectionIndex, 2), "pollock.draw"));
    popMatrix();
  }
  
  /**
   * Draw the possible directions in the origin, with its alpha representing the projection of the activated direction.
   */
  private void drawProjectionsInPossibleDirectionsInTheOrigin(){
    int directionIndex = 0;
    for(int altitudeIndex=0; altitudeIndex<this.numberOfAltitudes; altitudeIndex++){
      for(int azimuthIndex=0; azimuthIndex<this.numberOfAzimuths; azimuthIndex++){
        pushMatrix();
        strokeWeight(5);
        float colorIntensity = max(0, map((float)projectionInEachPossibleDirectionVector.get(directionIndex, 0), 0, 1, 0, 255));
        //println("color intensity: "+ colorIntensity);
        stroke(100, 0, 200, colorIntensity);
        PVector directionToDraw = new PVector((float) this.possibleDirections.get(directionIndex, 0), (float) this.possibleDirections.get(directionIndex, 1), (float) this.possibleDirections.get(directionIndex, 2));
        line(0, 0, 0, reScaleX(directionToDraw.x, "pollock.draw"),
                      reScaleY(directionToDraw.y, "pollock.draw"),
                      reScaleZ(directionToDraw.z, "pollock.draw"));
        directionIndex++;
        popMatrix();
      }
    }
  }
  
  /**
   * Draw the possible directions in the origin.
   */
  private void drawPossibleDirectionsInTheOrigin(){
    int directionIndex = 0;
    for(int altitudeIndex=0; altitudeIndex<this.numberOfAltitudes; altitudeIndex++){
      for(int azimuthIndex=0; azimuthIndex<this.numberOfAzimuths; azimuthIndex++){
        pushMatrix();
        strokeWeight(5);
        stroke(0, 0, 0, 255);
        PVector directionToDraw = new PVector((float) this.possibleDirections.get(directionIndex, 0), (float) this.possibleDirections.get(directionIndex, 1), (float) this.possibleDirections.get(directionIndex, 2));
        line(0, 0, 0, reScaleX(directionToDraw.x, "pollock.draw"),
                      reScaleY(directionToDraw.y, "pollock.draw"),
                      reScaleZ(directionToDraw.z, "pollock.draw"));
        directionIndex++;
        popMatrix();
      }
    }
  }
  
  /**
   * Draw the possible directions in the origin.
   */
  private void drawPossibleDirectionsInTheBody(){
    int directionIndex = 0;
    for(int altitudeIndex=0; altitudeIndex<this.numberOfAltitudes; altitudeIndex++){
      for(int azimuthIndex=0; azimuthIndex<this.numberOfAzimuths; azimuthIndex++){
        pushMatrix();
        strokeWeight(2);
        stroke(100, 0, 200, 255);
        translate(reScaleX(this.spineShoulderJoint.estimatedPosition.x, "pollock.draw"),
                  reScaleY(this.spineShoulderJoint.estimatedPosition.y, "pollock.draw"),
                  reScaleZ(this.spineShoulderJoint.estimatedPosition.z, "pollock.draw"));
        PVector directionToDraw = new PVector((float) this.possibleDirections.get(directionIndex, 0), (float) this.possibleDirections.get(directionIndex, 1), (float) this.possibleDirections.get(directionIndex, 2));
        Quaternion directionToDrawQuaternion = new Quaternion(0, directionToDraw);
        PVector relativeDirectionToDraw = qMult(qConjugate(this.spineShoulderJoint.estimatedOrientation), qMult(directionToDrawQuaternion, this.spineShoulderJoint.estimatedOrientation)).vector;
        line(0, 0, 0, reScaleX(-relativeDirectionToDraw.x, "pollock.draw"),// esse sinal de menos é gambiarra
                      reScaleY(relativeDirectionToDraw.y, "pollock.draw"),
                      reScaleZ(relativeDirectionToDraw.z, "pollock.draw"));
        directionIndex++;
        popMatrix();
      }
    }
  }
}
