public class Joint{
  private Skeleton skeleton;
  private int jointId;
  private boolean isEndJoint = false;
  private Joint parentJoint;
  private Bone parentBone;
  private ArrayList<Bone> childBones = new ArrayList<Bone>(); // the joint sits at the beginning of these bones. (closer to SpineMid)
  private int trackingState;
  private PVector measuredPosition;
  private float averageDistanceBetweenMeasuredPositionAndEstimatedPosition;
  private float distanceBetweenMeasuredPositionAndEstimatedPositionStandardDeviation; 
  private PVector estimatedPosition;
  private PVector estimatedVelocity;
  private PVector estimatedAcceleration;
  private Quaternion measuredOrientation;
  private float averageAngleBetweenMeasuredOrientationAndEstimatedOrientation; // theta
  private float angleBetweenMeasuredOrientationAndEstimatedOrientationStandardDeviation; // std of theta: angle between measuredOrientation and estimatedOrientation. 
  private Quaternion estimatedOrientation;
  private Quaternion previousEstimatedOrientation;
  private PVector estimatedDirectionX;
  private PVector estimatedDirectionY;
  private PVector estimatedDirectionZ;
  private PVector measuredDirectionX;
  private PVector measuredDirectionY;
  private PVector measuredDirectionZ;
  
  public Joint(int jointId, KJoint kJoint, Skeleton skeleton){
    this.skeleton = skeleton;
    this.jointId = jointId;
    if(this.jointId==3 || this.jointId==15 || this.jointId==19 || this.jointId>20){
      this.isEndJoint = true;
    }
    this.receiveNewMeasurements(kJoint);
    this.estimatedPosition = this.measuredPosition;
    this.averageDistanceBetweenMeasuredPositionAndEstimatedPosition = 0.05; // measurement error in meters
    this.distanceBetweenMeasuredPositionAndEstimatedPositionStandardDeviation = 0.025; // in meters. Pure guess!
    this.estimatedVelocity = new PVector(0,0,0);
    this.estimatedOrientation = this.measuredOrientation;
    this.previousEstimatedOrientation = this.estimatedOrientation;
    this.averageAngleBetweenMeasuredOrientationAndEstimatedOrientation = 1; // in radians. Pure guess! 
    this.angleBetweenMeasuredOrientationAndEstimatedOrientationStandardDeviation = 1; // em radianos. Pure guess!
  }
  
  public void addChildBone(Bone childBone){
    this.childBones.add(childBone);
  }
  
  public void setParentJoint(Joint parentJoint){
    this.parentJoint = parentJoint;
  }
  
  public void setParentBone(Bone parentBone){
    this.parentBone = parentBone;
  }
  
  public void receiveNewMeasurements(KJoint kjoint){
    this.trackingState = kjoint.getState();
    if(kjoint.getPosition().z > 0.01){ // discard impossible measurements
      this.measuredPosition = kjoint.getPosition();
      this.measuredOrientation = new Quaternion(kjoint.getOrientation());
    } else { // replace impossible measurements with prediction.
      this.measuredPosition = PVector.add(this.estimatedPosition, PVector.mult(this.estimatedVelocity, this.skeleton.scene.currentDeltaT));
      this.measuredOrientation = qSlerp(this.previousEstimatedOrientation, this.estimatedOrientation, 1 + this.skeleton.scene.currentDeltaT/this.skeleton.scene.previousDeltaT);
      println("impossible measurement of joint "+ this.jointId+" received and discarded (too close to kinect): "+kjoint.getPosition());
    }
  }
  
  private float[] adjustConfidenceParametersByAlphaMultiplier(float[] confidenceParameters, float alphaMultiplier){
    float alphaAdjusted = confidenceParameters[0]*alphaMultiplier;
    float betaAdjusted = confidenceParameters[1] + (confidenceParameters[0]-alphaAdjusted)*confidenceParameters[1]/(confidenceParameters[1]+confidenceParameters[2]);
    float gammaAdjusted = confidenceParameters[2] + (confidenceParameters[0]-alphaAdjusted)*confidenceParameters[2]/(confidenceParameters[1]+confidenceParameters[2]);
    float[] adjustedConfidenceParameters = {alphaAdjusted, betaAdjusted, gammaAdjusted};
    return adjustedConfidenceParameters;
  }  
  
  private float[] adjustConfidenceParametersByTrackingState(float[] confidenceParameters, int trackingState){
    float alphaAdjusted;
    if(trackingState == 2){ // If joint is tracked
      alphaAdjusted = confidenceParameters[0];
    }
    else if(trackingState == 1){ // If joint is inferred
      alphaAdjusted = confidenceParameters[0]/2;
    }
    else{ //(trackingState == 0) // If joint is not tracked
      alphaAdjusted = confidenceParameters[0]/4;
    }
    float betaAdjusted = confidenceParameters[1] + (confidenceParameters[0]-alphaAdjusted)*confidenceParameters[1]/(confidenceParameters[1]+confidenceParameters[2]);
    float gammaAdjusted = confidenceParameters[2] + (confidenceParameters[0]-alphaAdjusted)*confidenceParameters[2]/(confidenceParameters[1]+confidenceParameters[2]);
    float[] adjustedConfidenceParameters = {alphaAdjusted, betaAdjusted, gammaAdjusted};
    return adjustedConfidenceParameters;
  }

  private void updateOrientation(float[] confidenceParameters, float currentDeltaT, float previousDeltaT, float dampingFactor){
    float measuredAngleBetweenMeasuredOrientationAndEstimatedOrientation = angleBetweenQuaternions(this.measuredOrientation, this.estimatedOrientation);
    this.averageAngleBetweenMeasuredOrientationAndEstimatedOrientation = lerp(this.averageAngleBetweenMeasuredOrientationAndEstimatedOrientation, measuredAngleBetweenMeasuredOrientationAndEstimatedOrientation, confidenceParameters[0]);
    this.angleBetweenMeasuredOrientationAndEstimatedOrientationStandardDeviation = lerp(this.angleBetweenMeasuredOrientationAndEstimatedOrientationStandardDeviation, abs(measuredAngleBetweenMeasuredOrientationAndEstimatedOrientation-this.averageAngleBetweenMeasuredOrientationAndEstimatedOrientation), confidenceParameters[0]);
    float orientationStep = howCloseToTheMean(measuredAngleBetweenMeasuredOrientationAndEstimatedOrientation, this.averageAngleBetweenMeasuredOrientationAndEstimatedOrientation, this.angleBetweenMeasuredOrientationAndEstimatedOrientationStandardDeviation); 
    Quaternion predictedCurrentOrientation = qSlerp(this.previousEstimatedOrientation, this.estimatedOrientation, 1 + dampingFactor*currentDeltaT/previousDeltaT);
    Quaternion newEstimatedOrientation = qSlerp(predictedCurrentOrientation, this.measuredOrientation, orientationStep*confidenceParameters[0]);
    this.previousEstimatedOrientation = this.estimatedOrientation;
    this.estimatedOrientation = newEstimatedOrientation;
  }
  
  private void updatePosition(float[] confidenceParameters, float currentDeltaT, float dampingFactor){
    float distanceBetweenMeasuredPositionAndEstimatedPosition = PVector.sub(this.measuredPosition, this.estimatedPosition).mag();
    this.averageDistanceBetweenMeasuredPositionAndEstimatedPosition = lerp(this.averageDistanceBetweenMeasuredPositionAndEstimatedPosition, distanceBetweenMeasuredPositionAndEstimatedPosition, confidenceParameters[0]);
    this.distanceBetweenMeasuredPositionAndEstimatedPositionStandardDeviation = lerp(this.distanceBetweenMeasuredPositionAndEstimatedPositionStandardDeviation, abs(distanceBetweenMeasuredPositionAndEstimatedPosition-this.averageDistanceBetweenMeasuredPositionAndEstimatedPosition), confidenceParameters[0]);
    float measuredPositionConfidence = howCloseToTheMean(distanceBetweenMeasuredPositionAndEstimatedPosition, this.averageDistanceBetweenMeasuredPositionAndEstimatedPosition, this.distanceBetweenMeasuredPositionAndEstimatedPositionStandardDeviation); 
    
    float[] adjustedConfidenceParameters = adjustConfidenceParametersByAlphaMultiplier(confidenceParameters, measuredPositionConfidence);
    if(this.jointId!=1){ // If this joint has parentBone
      adjustedConfidenceParameters = adjustConfidenceParametersByAlphaMultiplier(adjustedConfidenceParameters, this.parentBone.measuredLengthConfidence);
    }
    
    PVector newEstimatedPosition;
    if(jointId == 1){ // SpineMid
      newEstimatedPosition = PVector.lerp(PVector.add(this.estimatedPosition, PVector.mult(this.estimatedVelocity, dampingFactor*currentDeltaT)), this.measuredPosition, adjustedConfidenceParameters[0]);
    } 
    else{ // Common Joints
      newEstimatedPosition = PVector.add(this.estimatedPosition, PVector.mult(this.estimatedVelocity, dampingFactor*currentDeltaT)).mult(adjustedConfidenceParameters[1])
                                            .add(PVector.add(this.parentJoint.estimatedPosition, PVector.mult(this.parentBone.currentEstimatedDirection, this.parentBone.estimatedLength)).mult(adjustedConfidenceParameters[2]))
                                            .add(PVector.mult(this.measuredPosition, adjustedConfidenceParameters[0]));
    }
    PVector newEstimatedVelocity = PVector.sub(newEstimatedPosition, this.estimatedPosition).div(currentDeltaT);
    this.estimatedAcceleration = PVector.sub(newEstimatedVelocity, this.estimatedVelocity).div(currentDeltaT);
    this.estimatedVelocity = newEstimatedVelocity;
    this.estimatedPosition = newEstimatedPosition;
    /*
    if(this.jointId == FOOT_LEFT){
      println("LeftFoot accel: "+this.estimatedAcceleration.mag());
      println("LeftFoot velocity: "+this.estimatedVelocity.mag());
    }*/
  }
  
  public void calculateEstimates(float[] confidenceParameters, float currentDeltaT, float previousDeltaT, float dampingFactor){
    float[] adjustedConfidenceParameters = this.adjustConfidenceParametersByTrackingState(confidenceParameters, this.trackingState);
    this.updatePosition(adjustedConfidenceParameters, currentDeltaT, dampingFactor);
    if(!this.isEndJoint){
      this.updateOrientation(adjustedConfidenceParameters, currentDeltaT, previousDeltaT, dampingFactor);
      this.calculateCoordinateSystemOrientation();
    }
    for(Bone childBone:this.childBones){// Continue Chained Update, calling next bones:      
      childBone.update(confidenceParameters, currentDeltaT, previousDeltaT, dampingFactor); 
    }    
  }
  
  public void drawPosition(color colorEstimated){
    strokeWeight(1);
    sphereDetail(6);
    if(this.trackingState == 2){ // normal sphere if tracked
      stroke(colorEstimated);
      fill(colorEstimated);
    } else if(this.trackingState == 1){ // black contour if inferred
      stroke(color(0, 0, 0, 170)); 
      fill(colorEstimated);
    } else{ // black fill and controur if not tracked
      stroke(color(0, 0, 0, 170)); 
      fill(color(0,0,0,170));
    }
    pushMatrix();
    translate(reScaleX(this.estimatedPosition.x),
              reScaleY(this.estimatedPosition.y),
              reScaleZ(this.estimatedPosition.z));
    sphere(3);
    popMatrix();
  }
  
  private void calculateCoordinateSystemOrientation(){
    this.estimatedDirectionX = qMult(qMult(this.estimatedOrientation, new Quaternion(0, 1, 0, 0)), qConjugate(this.estimatedOrientation)).vector; 
    this.estimatedDirectionY = qMult(qMult(this.estimatedOrientation, new Quaternion(0, 0, 1, 0)), qConjugate(this.estimatedOrientation)).vector; 
    this.estimatedDirectionZ = qMult(qMult(this.estimatedOrientation, new Quaternion(0, 0, 0, 1)), qConjugate(this.estimatedOrientation)).vector; 
    this.measuredDirectionX = qMult(qMult(this.measuredOrientation, new Quaternion(0, 1, 0, 0)), qConjugate(this.measuredOrientation)).vector; 
    this.measuredDirectionY = qMult(qMult(this.measuredOrientation, new Quaternion(0, 0, 1, 0)), qConjugate(this.measuredOrientation)).vector; 
    this.measuredDirectionZ = qMult(qMult(this.measuredOrientation, new Quaternion(0, 0, 0, 1)), qConjugate(this.measuredOrientation)).vector; 
  }
  
  public float distanceToFloor(){ // Shall be deprecated
    return this.skeleton.scene.floor.plane.distanceTo(this.estimatedPosition);
  }
  
  public void draw(color colorEstimated, boolean measuredSkeleton, boolean jointOrientation){
    this.drawPosition(colorEstimated);
    if(jointOrientation && !this.isEndJoint){
      this.drawOrientation(true, measuredSkeleton); // estimated, measured
    }
  }
  
  public void drawOrientation(boolean drawEstimated, boolean drawMeasured){ // X:Red, Y:Green, Z:Blue
    float size = 15;
    pushMatrix();
    translate(reScaleX(this.estimatedPosition.x), reScaleY(this.estimatedPosition.y), reScaleZ(this.estimatedPosition.z));
    if(drawEstimated){
      strokeWeight(5);
      stroke(255, 0, 0, 170);
      line(0, 0, 0, size*this.estimatedDirectionX.x, size*this.estimatedDirectionX.y, size*this.estimatedDirectionX.z);
      stroke(0, 255, 0, 170);
      line(0, 0, 0, size*this.estimatedDirectionY.x, size*this.estimatedDirectionY.y, size*this.estimatedDirectionY.z);
      stroke(0, 0, 255, 170);
      line(0, 0, 0, size*this.estimatedDirectionZ.x, size*this.estimatedDirectionZ.y, size*this.estimatedDirectionZ.z);
    }
    if(drawMeasured){
      strokeWeight(2);
      stroke(255, 0, 0, 85);
      line(0, 0, 0, size*this.measuredDirectionX.x, size*this.measuredDirectionX.y, size*this.measuredDirectionX.z);
      stroke(0, 255, 0, 85);
      line(0, 0, 0, size*this.measuredDirectionY.x, size*this.measuredDirectionY.y, size*this.measuredDirectionY.z);
      stroke(0, 0, 255, 85);
      line(0, 0, 0, size*this.measuredDirectionZ.x, size*this.measuredDirectionZ.y, size*this.measuredDirectionZ.z);
    }
    popMatrix();
  }
}
