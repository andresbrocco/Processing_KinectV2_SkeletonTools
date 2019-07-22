/**
 * Pollock: Trigger Affordance that quantifies the "throwing" movement of each hand.
 */
public class Pollock{
  private final float speedThreshold = 0.75;
  private final int numberOfAltitudes = 3; // max: 9
  private final float altitudeDiscretization = PI/(this.numberOfAltitudes+1); // azimuth discretization (in radians)
  private final int numberOfAzimuths = 4; // max: 9
  private final float azimuthDiscretization = TWO_PI/this.numberOfAzimuths; // azimuth discretization (in radians)
  private final int forwardIsBetweenPossibleDirections = 0; // 1:forward is between possible directions. 0:forward is a possible direction.
  private int fadeOutTime = 1000; // millisseconds
  private String whichHand; // LEFT or RIGHT
  private Joint handJoint;
  private Joint shoulderJoint;
  private Joint headJoint;
  private Joint spineShoulderJoint;
  private PVector shoulderToHandPosition;
  private PVector shoulderToHandDirection; // Normalized shoulderToHandPosition
  private PVector shoulderToHandVelocity = new PVector(0, 0, 0);
  private float shoulderToHandSpeed = 0;
  private float shoulderToHandSpeedAdjustedByParalelism = 0;
  private boolean shoulderToHandSpeedWasAboveThreshold = false;
  private boolean shoulderToHandSpeedIsAboveThreshold = false;
  private Matrix possibleDirectionsMatrix = new Matrix(numberOfAltitudes*numberOfAzimuths, 3);
  private PVector headToHandPosition;
  private PVector headToHandDirection; // vector from head to hand normalized.
  private PVector headToHandDirectionRelativeToShoulder; // vector from head to hand normalized, relative to shoulder coordinate system.
  private int activationTime; // millisseconds
  private int activationDirectionIndex = 0;
  private int activationDirectionCode; // dozen: azimuth. unit: altitude.
  
  Pollock(Skeleton skeleton, String whichHand){
    this.whichHand = whichHand;
    switch(this.whichHand){
      case "LEFT":
        this.handJoint = skeleton.joints[HAND_LEFT];
        this.shoulderJoint = skeleton.joints[SHOULDER_LEFT];
        break;
      case "RIGHT":
        this.handJoint = skeleton.joints[HAND_RIGHT];
        this.shoulderJoint = skeleton.joints[SHOULDER_RIGHT];
        break;
    }
    this.headJoint = skeleton.joints[HEAD];
    this.spineShoulderJoint = skeleton.joints[SPINE_SHOULDER];
    this.buildPossibleDirectionsMatrix();
  }
  
  /**
   * Build a Matrix containing the vectors for all possible directions of the pollock, based on the chosen number of azimuths and altitudes.
   */
  private void buildPossibleDirectionsMatrix(){
    int directionIndex = 0;
    for(int altitudeIndex=0; altitudeIndex<this.numberOfAltitudes; altitudeIndex++){
      for(int azimuthIndex=0; azimuthIndex<this.numberOfAzimuths; azimuthIndex++){
        this.possibleDirectionsMatrix.set(directionIndex, 0, sin((altitudeIndex+1)*this.altitudeDiscretization)*sin((azimuthIndex+((float)forwardIsBetweenPossibleDirections)/2)*this.azimuthDiscretization));
        this.possibleDirectionsMatrix.set(directionIndex, 1, cos((altitudeIndex+1)*this.altitudeDiscretization));
        this.possibleDirectionsMatrix.set(directionIndex, 2, sin((altitudeIndex+1)*this.altitudeDiscretization)*cos((azimuthIndex+((float)forwardIsBetweenPossibleDirections)/2)*this.azimuthDiscretization));
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
    float paralelismFactor = abs(this.shoulderToHandSpeed)/this.shoulderToHandVelocity.mag();
    this.shoulderToHandSpeedAdjustedByParalelism = this.shoulderToHandSpeed*paralelismFactor;
    println("shoulderToHandSpeedAdjustedByParalelism: "+this.shoulderToHandSpeedAdjustedByParalelism);
    this.shoulderToHandSpeedIsAboveThreshold = this.shoulderToHandSpeedAdjustedByParalelism > this.speedThreshold;
    
    if(this.shoulderToHandSpeedIsAboveThreshold){
      this.headToHandPosition = PVector.sub(this.handJoint.estimatedPosition, this.headJoint.estimatedPosition);
    } else if(this.shoulderToHandSpeedWasAboveThreshold){ // Pollock is activated here
      this.activationTime = millis();
      this.findDirection();
    }
    this.shoulderToHandSpeedWasAboveThreshold = this.shoulderToHandSpeedIsAboveThreshold;
  }
  
  /**
   * Find the possible direction that has the largest dot product with the activation direction.   
   */
  private void findDirection(){
    this.headToHandDirection = PVector.div(this.headToHandPosition, headToHandPosition.mag());
    this.headToHandDirectionRelativeToShoulder = new PVector(PVector.dot(this.headToHandDirection, this.spineShoulderJoint.estimatedDirectionX), 
                                                             PVector.dot(this.headToHandDirection, this.spineShoulderJoint.estimatedDirectionY), 
                                                             PVector.dot(this.headToHandDirection, this.spineShoulderJoint.estimatedDirectionZ));
    Matrix headToHandDirectionRelativeToShoulderVector = new Matrix(new double[] {this.headToHandDirectionRelativeToShoulder.x, 
                                                                                  this.headToHandDirectionRelativeToShoulder.y, 
                                                                                  this.headToHandDirectionRelativeToShoulder.z}, 3);
    Matrix projectionInEachPossibleDirectionVector = this.possibleDirectionsMatrix.times(headToHandDirectionRelativeToShoulderVector);
    double max = 0;
    int possibleDirectionIndex = 0;
    for(int possibleDirectionAltitude=0; possibleDirectionAltitude<this.numberOfAltitudes; possibleDirectionAltitude++){
      for(int possibleDirectionAzimuth=0; possibleDirectionAzimuth<this.numberOfAzimuths; possibleDirectionAzimuth++){
        if(projectionInEachPossibleDirectionVector.get(possibleDirectionIndex, 0) > max){
          this.activationDirectionIndex = possibleDirectionIndex;
          this.activationDirectionCode = possibleDirectionAzimuth*10 + possibleDirectionAltitude;
          max = projectionInEachPossibleDirectionVector.get(possibleDirectionIndex, 0);
        }
        possibleDirectionIndex++;
      }
    }
    //println("pollockCode: " + this.activationDirectionCode);
  }
  
  /**
   * Draw the representations of the pollock affordance.
   */
  private void draw(boolean drawTriggerVector, boolean drawPossibleDirectionsInTheOrigin, boolean drawHeadToHandPosition){
    if(drawTriggerVector) this.drawTriggerVector();
    if(drawPossibleDirectionsInTheOrigin) this.drawPossibleDirectionsInTheOrigin(0.5);
    if(millis()-this.activationTime < this.fadeOutTime){
      if(drawHeadToHandPosition) this.drawHeadToHandPosition();
      this.drawActivationDirectionInTheOrigin(0.5);
    }
  }
  
  /**
   * Draw the vector from shoulder to arm. Its stroke alpha represents the velocity of the pollock.
   */
  private void drawTriggerVector(){
    pushMatrix();
    strokeWeight(5);
    stroke(0, 0, 0, max(0, min(255, map(this.shoulderToHandSpeedAdjustedByParalelism, 0, 2, 0, 255))));
    translate(reScaleX(this.shoulderJoint.estimatedPosition.x, "pollock.draw"),
              reScaleY(this.shoulderJoint.estimatedPosition.y, "pollock.draw"),
              reScaleZ(this.shoulderJoint.estimatedPosition.z, "pollock.draw"));
    PVector shoulderToHand = PVector.sub(this.handJoint.estimatedPosition, this.shoulderJoint.estimatedPosition);
    line(0, 0, 0, reScaleX(shoulderToHand.x, "pollock.draw"), 
                  reScaleY(shoulderToHand.y, "pollock.draw"), 
                  reScaleZ(shoulderToHand.z, "pollock.draw"));
    popMatrix();
  }
  
  /**
   * Draw the vector from head to hand, which is the direction of the activation.
   */
  private void drawHeadToHandPosition(){
    pushMatrix();
    strokeWeight(5);
    stroke(0, 0, 0, 128);
    translate(reScaleX(this.headJoint.estimatedPosition.x, "pollock.draw"),
              reScaleY(this.headJoint.estimatedPosition.y, "pollock.draw"),
              reScaleZ(this.headJoint.estimatedPosition.z, "pollock.draw")); 
    line(0, 0, 0, reScaleX(this.headToHandPosition.x, "pollock.draw"), 
                  reScaleY(this.headToHandPosition.y, "pollock.draw"), 
                  reScaleZ(this.headToHandPosition.z, "pollock.draw"));
    popMatrix();
  }
  
  /**
   * Draw the activated possible direction in the origin.
   */
  private void drawActivationDirectionInTheOrigin(float size){
    PVector possibleDirectionActivated = (new PVector((float)this.possibleDirectionsMatrix.get(this.activationDirectionIndex, 0), 
                                                      (float)this.possibleDirectionsMatrix.get(this.activationDirectionIndex, 1), 
                                                      (float)this.possibleDirectionsMatrix.get(this.activationDirectionIndex, 2))).mult(size);
    pushMatrix();
    strokeWeight(5);
    // Draw headToHandDirectionRelativeToShoulder:
    stroke(0, 0, 0, 255);
    line(0, 0, 0, size*reScaleX(this.headToHandDirectionRelativeToShoulder.x, "pollock.draw"), 
                  size*reScaleY(this.headToHandDirectionRelativeToShoulder.y, "pollock.draw"), 
                  size*reScaleZ(this.headToHandDirectionRelativeToShoulder.z, "pollock.draw"));
    
    stroke(100, 0, 200, 255);    
    // Draw shell:
    Quaternion azimuthClockwiseRotation =      axisAngleToQuaternion(0, 1, 0, this.azimuthDiscretization/2); // clockwise half rotation
    Quaternion azimuthAntiClockwiseRotation =  axisAngleToQuaternion(0, 1, 0, -this.azimuthDiscretization/2); // anticlockwise half rotation
    Quaternion altitudeClockwiseRotation =     axisAngleToQuaternion(rotateVector(new PVector(possibleDirectionActivated.x, 0, possibleDirectionActivated.z), new PVector(0, 1, 0), HALF_PI), this.altitudeDiscretization/2); // clockwise half rotation
    Quaternion altitudeAntiClockwiseRotation = axisAngleToQuaternion(rotateVector(new PVector(possibleDirectionActivated.x, 0, possibleDirectionActivated.z), new PVector(0, 1, 0), HALF_PI), -this.altitudeDiscretization/2); // anticlockwise half rotation
    
    PVector vertex1 = rotateVector(rotateVector(possibleDirectionActivated, altitudeClockwiseRotation)    , azimuthClockwiseRotation);
    PVector vertex2 = rotateVector(rotateVector(possibleDirectionActivated, altitudeAntiClockwiseRotation), azimuthClockwiseRotation);
    PVector vertex3 = rotateVector(rotateVector(possibleDirectionActivated, altitudeAntiClockwiseRotation), azimuthAntiClockwiseRotation);
    PVector vertex4 = rotateVector(rotateVector(possibleDirectionActivated, altitudeClockwiseRotation)    , azimuthAntiClockwiseRotation);
    
    float fadeOutFactor = 1-(float)((millis()-this.activationTime)/(float)this.fadeOutTime);
    fill(100, 0, 200, 255*fadeOutFactor);
    beginShape();
    vertex(vertex1, "pollock.draw");
    vertex(vertex2, "pollock.draw");
    vertex(vertex3, "pollock.draw");
    vertex(vertex4, "pollock.draw");
    endShape(CLOSE);
    popMatrix();
  }
  
  /**
   * Draw the possible directions in the origin.
   */
  private void drawPossibleDirectionsInTheOrigin(float size){
    int directionIndex = 0;
    for(int altitudeIndex=0; altitudeIndex<this.numberOfAltitudes; altitudeIndex++){
      for(int azimuthIndex=0; azimuthIndex<this.numberOfAzimuths; azimuthIndex++){
        pushMatrix();
        strokeWeight(2);
        stroke(0, 0, 0, 40);
        noFill();
        PVector directionToDraw = (new PVector((float) this.possibleDirectionsMatrix.get(directionIndex, 0), 
                                               (float) this.possibleDirectionsMatrix.get(directionIndex, 1), 
                                               (float) this.possibleDirectionsMatrix.get(directionIndex, 2))).mult(size);
        
        // Draw shell:
        Quaternion azimuthClockwiseRotation =      axisAngleToQuaternion(0, 1, 0, this.azimuthDiscretization/2); // clockwise half rotation
        Quaternion azimuthAntiClockwiseRotation =  axisAngleToQuaternion(0, 1, 0, -this.azimuthDiscretization/2); // clockwise half rotation
        Quaternion altitudeClockwiseRotation =     axisAngleToQuaternion(rotateVector(new PVector(directionToDraw.x, 0, directionToDraw.z), new PVector(0, 1, 0), HALF_PI), this.altitudeDiscretization/2); // clockwise half rotation
        Quaternion altitudeAntiClockwiseRotation = axisAngleToQuaternion(rotateVector(new PVector(directionToDraw.x, 0, directionToDraw.z), new PVector(0, 1, 0), HALF_PI), -this.altitudeDiscretization/2); // clockwise half rotation
        
        PVector vertex1 = rotateVector(rotateVector(directionToDraw, altitudeClockwiseRotation)    , azimuthClockwiseRotation);
        PVector vertex2 = rotateVector(rotateVector(directionToDraw, altitudeAntiClockwiseRotation), azimuthClockwiseRotation);
        PVector vertex3 = rotateVector(rotateVector(directionToDraw, altitudeAntiClockwiseRotation), azimuthAntiClockwiseRotation);
        PVector vertex4 = rotateVector(rotateVector(directionToDraw, altitudeClockwiseRotation)    , azimuthAntiClockwiseRotation);
        
        beginShape();
        vertex(vertex1, "pollock.draw");
        vertex(vertex2, "pollock.draw");
        vertex(vertex3, "pollock.draw");
        vertex(vertex4, "pollock.draw");
        endShape(CLOSE);
        popMatrix();
        directionIndex++;
      }
    }
  }
}
