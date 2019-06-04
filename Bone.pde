/**
 * A Bone object contains information about its present and past orientation and length. It also knows who is its parent and child joint.
 */
public class Bone{
  private Skeleton skeleton;
  private int boneId; 
  private Joint parentJoint;
  private Joint childJoint;
  private int trackingState; // multiplication of trackingStates of its joints. If 4: both joints tracked. If 2: one joint tracked, other inferred. If 1: both inferred. If 0: one not tracked. 
  private float measuredLength;
  private float averageMeasuredLength;
  private float measuredLengthStandardDeviation; //std of measuredLength
  private float measuredLengthConfidence;
  private float estimatedLength;
  private PVector estimatedPosition;
  private PVector measuredDirection;
  private float averageAngleBetweenMeasuredDirectionAndEstimatedDirection; // theta
  private float angleBetweenMeasuredDirectionAndEstimatedDirectionStandardDeviation; //std of theta 
  private PVector currentEstimatedDirection;
  private PVector previousEstimatedDirection;
  private PVector relativeOrientation; // roll, pitch, yaw
  
  public Bone(Skeleton skeleton, int boneId, Joint parentJoint, Joint childJoint){ // The bones must be initialized after all the joints, because its constructor acess values from its joints.
    this.skeleton = skeleton;
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
  
/**
 * Updates its estimatives and call the child joint to be updated.
 */
  public void update(float[] confidenceParameters){
    this.trackingState = this.parentJoint.trackingState*this.childJoint.trackingState;
    this.updateLength(confidenceParameters[0]/10); // the length of the bones should not be too sensitive to new measurements.
    this.estimatedPosition = PVector.lerp(this.parentJoint.estimatedPosition, this.childJoint.estimatedPosition, 0.5);
    this.updateDirection(confidenceParameters[0]);
    this.relativeOrientation = calculateRelativeOrientation(this.parentJoint.estimatedOrientation, this.childJoint.estimatedOrientation);
    this.childJoint.update(confidenceParameters); // Continue Chained Update, calling next joint
  }
  
/**
 * Estimate its length based on previous estimate and current measurement.
 * First, it judges if the new measurement is an outlier or not, by calculating how close to the mean it is.
 * Then it uses this step size to calculate the new estimatedLength using the previousEstimatedLength and the current measurement. 
 * @param alpha (basis confidence on new measurements)
 */
  private void updateLength(float alpha){
    this.measuredLength = PVector.sub(this.parentJoint.measuredPosition, this.childJoint.measuredPosition).mag();
    float alphaAdjustedByTrackingState = adjustAlphaByTrackingState(alpha, this.trackingState);
    this.averageMeasuredLength = lerp(this.averageMeasuredLength, this.measuredLength, pow(alphaAdjustedByTrackingState, this.skeleton.responseTradeoff));
    this.measuredLengthStandardDeviation = lerp(this.measuredLengthStandardDeviation, abs(this.measuredLength-this.averageMeasuredLength), pow(alphaAdjustedByTrackingState, this.skeleton.responseTradeoff));
    this.measuredLengthConfidence = howCloseToTheMean(this.measuredLength, this.averageMeasuredLength, this.measuredLengthStandardDeviation);
    float alphaAdjustedByTrackingStateAndConfidence = alphaAdjustedByTrackingState*this.measuredLengthConfidence;
    this.estimatedLength = lerp(this.estimatedLength, this.measuredLength, alphaAdjustedByTrackingStateAndConfidence);
  }
  
/**
 * Iterates one step on the estimation of the current direction.
 * First, it judges if the new measurement is an outlier or not, by calculating how close to the mean it is.
 * Then it uses this step size to calculate the new estimatedDirection using the previousEstimatedDirection and the current measurement. 
 * It uses slerp extrapolation (step>1) to account for direction velocity.
 * @param alpha basis confidence on new measurements 
 */
  public void updateDirection(float alpha){
    this.measuredDirection = PVector.sub(this.childJoint.measuredPosition, this.parentJoint.measuredPosition).div(this.measuredLength);
    float angleBetweenMeasuredDirectionAndEstimatedDirection = PVector.angleBetween(this.measuredDirection, this.currentEstimatedDirection);
    float alphaAdjustedByTrackingState = this.adjustAlphaByTrackingState(alpha, this.trackingState);
    this.averageAngleBetweenMeasuredDirectionAndEstimatedDirection = lerp(this.averageAngleBetweenMeasuredDirectionAndEstimatedDirection, angleBetweenMeasuredDirectionAndEstimatedDirection, pow(alphaAdjustedByTrackingState, this.skeleton.responseTradeoff));
    this.angleBetweenMeasuredDirectionAndEstimatedDirectionStandardDeviation = lerp(this.angleBetweenMeasuredDirectionAndEstimatedDirectionStandardDeviation, abs(angleBetweenMeasuredDirectionAndEstimatedDirection-this.averageAngleBetweenMeasuredDirectionAndEstimatedDirection), pow(alphaAdjustedByTrackingState, this.skeleton.responseTradeoff));
    float measuredDirectionConfidence = howCloseToTheMean(angleBetweenMeasuredDirectionAndEstimatedDirection, this.averageAngleBetweenMeasuredDirectionAndEstimatedDirection, this.angleBetweenMeasuredDirectionAndEstimatedDirectionStandardDeviation);
    float alphaAdjustedByTrackingStateAndConfidence = alphaAdjustedByTrackingState*measuredDirectionConfidence;
    PVector auxiliar = slerp(slerp(this.previousEstimatedDirection, this.currentEstimatedDirection, 1+this.skeleton.dampingFactor*this.skeleton.scene.currentDeltaT/this.skeleton.scene.previousDeltaT), this.measuredDirection, alphaAdjustedByTrackingStateAndConfidence);
    this.previousEstimatedDirection = this.currentEstimatedDirection;
    this.currentEstimatedDirection = auxiliar;
  }
  
/**
 * Adjust the confidence of the new measurement based on the trackingState of adjacent joints received from Kinect.
 * If both joints are tracked, alpha = alpha. 
 * If one tracked and one inferred, alpha = alpha/2.
 * If both inferred, alpha = alpha/4.
 * If at least one is not tracked, alpha = 0;
 * @param alpha (basis confidence of the new measurements)
 * @param trackingState received from kinect.
 * @return alphaAdjustedByTrackingState
 */
  private float adjustAlphaByTrackingState(float alpha, int trackingState){
    float alphaAdjustedByTrackingState;
    if(trackingState == 4){ // If both joints are tracked
      alphaAdjustedByTrackingState = alpha;
    }
    else if(trackingState == 2){ // If one tracked and one inferred
      alphaAdjustedByTrackingState = alpha/3;
    }
    else if(trackingState == 1){ // If both inferred
      alphaAdjustedByTrackingState = alpha/9;
    }
    else { // if(trackingState == 0)  // If at least one is not tracked
      alphaAdjustedByTrackingState = 0;
    }
    return alphaAdjustedByTrackingState;
  }
  
/**
 * Draw a line representing the bone, and a slice of a 3D pie representing its relative orientation.
 */
  public void draw(boolean drawMeasured, boolean drawBoneRelativeOrientation){
    this.drawBone(drawMeasured);
    if(drawBoneRelativeOrientation){
      this.drawRelativeOrientation();
    }
  }
  
/**
 * Draw a slice of a 3D pie representing its orientation relative to the parent joint.
 */
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
  
/**
 * Draw a line connecting child joint to parent joint.
 * @param drawMeasured boolean to enable drawing the raw data received from kinect.
 */
  public void drawBone(boolean drawMeasured){
    stroke(this.skeleton.colorEstimated); fill(this.skeleton.colorEstimated);
    strokeWeight(5);
    line(reScaleX(this.parentJoint.estimatedPosition.x), 
         reScaleY(this.parentJoint.estimatedPosition.y), 
         reScaleZ(this.parentJoint.estimatedPosition.z), 
         reScaleX(this.childJoint.estimatedPosition.x), 
         reScaleY(this.childJoint.estimatedPosition.y), 
         reScaleZ(this.childJoint.estimatedPosition.z));
    if(drawMeasured){
      stroke(this.skeleton.colorMeasured); fill(this.skeleton.colorMeasured);
      line(reScaleX(this.parentJoint.measuredPosition.x), 
           reScaleY(this.parentJoint.measuredPosition.y), 
           reScaleZ(this.parentJoint.measuredPosition.z), 
           reScaleX(this.childJoint.measuredPosition.x), 
           reScaleY(this.childJoint.measuredPosition.y), 
           reScaleZ(this.childJoint.measuredPosition.z));
    }
  }
}
