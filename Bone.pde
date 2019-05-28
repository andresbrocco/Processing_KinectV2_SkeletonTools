public class Bone{
  private int boneId; 
  private Joint parentJoint;
  private Joint childJoint;
  private int trackingState; // multiplication of trackingStates of its joints. If 4: both joints tracked. If 2: one joint tracked, other inferred. If 1: both inferred. If 0: one not tracked. 
  private float measuredLength;
  private float averageMeasuredLength;
  private float measuredLengthStandardDeviation; //std of measuredLength
  public float measuredLengthConfidence;
  private float estimatedLength;
  private PVector estimatedPosition;
  private PVector measuredDirection;
  private float averageAngleBetweenMeasuredDirectionAndEstimatedDirection; // theta
  private float angleBetweenMeasuredDirectionAndEstimatedDirectionStandardDeviation; //std of theta 
  private PVector currentEstimatedDirection;
  private PVector previousEstimatedDirection;
  private PVector relativeOrientation; // roll, pitch, yaw
  
  public Bone(int boneId, Joint parentJoint, Joint childJoint){ // The bones must be initialized after all the joints, because its constructor acess values from its joints. 
    this.boneId = boneId;
    this.parentJoint = parentJoint;
    this.childJoint = childJoint;
    this.trackingState = this.parentJoint.trackingState*this.childJoint.trackingState;
    this.measuredLength = PVector.sub(this.parentJoint.measuredPosition, this.childJoint.measuredPosition).mag();
    this.estimatedPosition = PVector.lerp(this.parentJoint.measuredPosition, this.childJoint.measuredPosition, 0.5);
    this.averageMeasuredLength = this.measuredLength;
    this.estimatedLength = this.measuredLength;
    this.measuredLengthStandardDeviation = this.measuredLength/2; // chute inicial 
    this.measuredDirection = PVector.sub(this.childJoint.measuredPosition, this.parentJoint.measuredPosition).div(this.measuredLength);
    this.currentEstimatedDirection = this.measuredDirection;
    this.previousEstimatedDirection = this.measuredDirection;
    this.averageAngleBetweenMeasuredDirectionAndEstimatedDirection = 1; // "initial guess", in radians.
    this.angleBetweenMeasuredDirectionAndEstimatedDirectionStandardDeviation = 1; // "initial guess", in radians.
    this.relativeOrientation = calculateRelativeOrientation(this.parentJoint.measuredOrientation, this.childJoint.measuredOrientation);
  }
  
  public void update(float[] confidenceParameters, float currentDeltaT, float previousDeltaT, float dampingFactor){
    this.trackingState = this.parentJoint.trackingState*this.childJoint.trackingState;
    this.estimateLength(confidenceParameters[0]/10); // the length of the bones should not be too sensitive to new measurements.
    this.estimatedPosition = PVector.lerp(this.parentJoint.estimatedPosition, this.childJoint.estimatedPosition, 0.5);
    this.estimateDirection(confidenceParameters[0], currentDeltaT, previousDeltaT, dampingFactor);
    this.relativeOrientation = calculateRelativeOrientation(this.parentJoint.currentEstimatedOrientation, this.childJoint.currentEstimatedOrientation);
    this.childJoint.calculateEstimates(confidenceParameters, currentDeltaT, previousDeltaT, dampingFactor); // Continue Chained Update, calling next joint
  }
  
  private void estimateLength(float alpha){
    this.measuredLength = PVector.sub(this.parentJoint.measuredPosition, this.childJoint.measuredPosition).mag();
    float alphaAdjustedByTrackingState = adjustAlphaByTrackingState(alpha, this.trackingState);
    this.averageMeasuredLength = lerp(this.averageMeasuredLength, this.measuredLength, alphaAdjustedByTrackingState);
    this.measuredLengthStandardDeviation = lerp(this.measuredLengthStandardDeviation, abs(this.measuredLength-this.averageMeasuredLength), alphaAdjustedByTrackingState);
    this.measuredLengthConfidence = howCloseToTheMean(this.measuredLength, this.averageMeasuredLength, this.measuredLengthStandardDeviation);
    float alphaAdjustedByTrackingStateAndConfidence = alphaAdjustedByTrackingState*this.measuredLengthConfidence;
    this.estimatedLength = lerp(this.estimatedLength, this.measuredLength, alphaAdjustedByTrackingStateAndConfidence);
  }
  
  private float adjustAlphaByTrackingState(float alpha, int trackingState){
    float alphaAdjustedByTrackingState;
    if(trackingState == 4){ // If both joints are tracked
      alphaAdjustedByTrackingState = alpha;
    }
    else if(trackingState == 2){ // If one tracked and one inferred
      alphaAdjustedByTrackingState = alpha/2;
    }
    else if(trackingState == 1){ // If both inferred
      alphaAdjustedByTrackingState = alpha/4;
    }
    else { // if(trackingState == 0)  // If at least one is not tracked
      alphaAdjustedByTrackingState = 0;
    }
    return alphaAdjustedByTrackingState;
  }
  
  public void estimateDirection(float alpha, float currentDeltaT, float previousDeltaT, float dampingFactor){
    this.measuredDirection = PVector.sub(this.childJoint.measuredPosition, this.parentJoint.measuredPosition).div(this.measuredLength);
    float angleBetweenMeasuredDirectionAndEstimatedDirection = PVector.angleBetween(this.measuredDirection, this.currentEstimatedDirection);
    float alphaAdjustedByTrackingState = this.adjustAlphaByTrackingState(alpha, this.trackingState);
    
    this.averageAngleBetweenMeasuredDirectionAndEstimatedDirection = lerp(this.averageAngleBetweenMeasuredDirectionAndEstimatedDirection, angleBetweenMeasuredDirectionAndEstimatedDirection, alphaAdjustedByTrackingState);
    this.angleBetweenMeasuredDirectionAndEstimatedDirectionStandardDeviation = lerp(this.angleBetweenMeasuredDirectionAndEstimatedDirectionStandardDeviation, abs(angleBetweenMeasuredDirectionAndEstimatedDirection-this.averageAngleBetweenMeasuredDirectionAndEstimatedDirection), alphaAdjustedByTrackingState);
    float measuredDirectionConfidence = howCloseToTheMean(angleBetweenMeasuredDirectionAndEstimatedDirection, this.averageAngleBetweenMeasuredDirectionAndEstimatedDirection, this.angleBetweenMeasuredDirectionAndEstimatedDirectionStandardDeviation);
    float alphaAdjustedByTrackingStateAndConfidence = alphaAdjustedByTrackingState*measuredDirectionConfidence;
    PVector auxiliar = slerp(slerp(this.previousEstimatedDirection, this.currentEstimatedDirection, 1+dampingFactor*currentDeltaT/previousDeltaT), this.measuredDirection, alphaAdjustedByTrackingStateAndConfidence);
    this.previousEstimatedDirection = this.currentEstimatedDirection;
    this.currentEstimatedDirection = auxiliar;
    
  }
  
  public void draw(color colorEstimated, color colorMeasured, boolean measuredSkeleton, boolean boneRelativeOrientation){
    this.drawBone(colorEstimated, colorMeasured, measuredSkeleton);
    if(boneRelativeOrientation){
      this.drawRelativeOrientation();
    }
  }
  
  public void drawRelativeOrientation(){ // X:Red, Y:Green, Z:Blue
  float size = 15;
    if(!this.childJoint.isEndJoint){
      pushMatrix();
      translate(reScaleX(this.estimatedPosition.x),
                reScaleY(this.estimatedPosition.y),
                reScaleZ(this.estimatedPosition.z));
      noStroke();
      fill(255, 0, 0, 128); 
      drawPie3D(this.parentJoint.estimatedDirectionX, this.childJoint.estimatedDirectionX, size);
      fill(0, 255, 0, 128); 
      drawPie3D(this.parentJoint.estimatedDirectionY, this.childJoint.estimatedDirectionY, size);
      fill(0, 0, 255, 128); 
      drawPie3D(this.parentJoint.estimatedDirectionZ, this.childJoint.estimatedDirectionZ, size);
      popMatrix();
    }
  }
  
  public void drawBone(color colorEstimated, color colorMeasured, boolean drawMeasuredBone){
    stroke(colorEstimated); fill(colorEstimated);
    strokeWeight(5);
    line(reScaleX(this.parentJoint.estimatedPosition.x), 
         reScaleY(this.parentJoint.estimatedPosition.y), 
         reScaleZ(this.parentJoint.estimatedPosition.z), 
         reScaleX(this.childJoint.estimatedPosition.x), 
         reScaleY(this.childJoint.estimatedPosition.y), 
         reScaleZ(this.childJoint.estimatedPosition.z));
    if(drawMeasuredBone){
      stroke(colorMeasured); fill(colorMeasured);
      line(reScaleX(this.parentJoint.measuredPosition.x), 
           reScaleY(this.parentJoint.measuredPosition.y), 
           reScaleZ(this.parentJoint.measuredPosition.z), 
           reScaleX(this.childJoint.measuredPosition.x), 
           reScaleY(this.childJoint.measuredPosition.y), 
           reScaleZ(this.childJoint.measuredPosition.z));
    }
  }
}
