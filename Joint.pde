/**
 * A Joint object contains information about its present and past orientation and position. It also knows who is its parent joint and bone.
 */
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
  private float averageAngleBetweenMeasuredOrientationAndPredictedCurrentOrientation; // theta
  private float angleBetweenMeasuredOrientationAndPredictedCurrentOrientationStandardDeviation; // std of theta: angle between measuredOrientation and estimatedOrientation. 
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
    this.averageAngleBetweenMeasuredOrientationAndPredictedCurrentOrientation = 1; // in radians. Pure guess! 
    this.angleBetweenMeasuredOrientationAndPredictedCurrentOrientationStandardDeviation = 1; // in radians. Pure guess!
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
  
/**
 * Get new joint measurements from kinect. Discard it if is an impossible measurement and replace it by an educated prediction.
 */
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
  
/**
 * Updates itself and call next bones to be updated.
 * @param confidenceParameters (alpha, beta, gamma).
 */
  public void update(float[] confidenceParameters){
    float[] adjustedConfidenceParameters = this.adjustConfidenceParametersByTrackingState(confidenceParameters, this.trackingState);
    this.updatePosition(adjustedConfidenceParameters);
    if(!this.isEndJoint){
      this.updateOrientation(adjustedConfidenceParameters);
      this.calculateCoordinateSystemOrientation();
    }
    for(Bone childBone:this.childBones){// Continue Chained Update, calling next bones:      
      childBone.update(confidenceParameters); 
    }    
  }
  
/**
 * Iterates one step on the estimation of the current position.
 * First, it judges if the new measurement is an outlier or not, by calculating how close to the mean it is.
 * Then it uses this step size to calculate the new estimatedPosition using the previousEstimatedPosition, velocity and the current measurement. 
 * @param confidenceParameters (alpha, beta, gamma).
 */
  private void updatePosition(float[] confidenceParameters){
    PVector newEstimatedPosition;
    float[] adjustedConfidenceParametersForresponseTradeoff = adjustConfidenceParametersByResponseTradeoff(confidenceParameters);
    if(jointId == 1){ // SpineMid
      newEstimatedPosition = PVector.add(this.estimatedPosition, PVector.mult(this.estimatedVelocity, this.skeleton.scene.currentDeltaT));
    } 
    else{ // Common Joints
      newEstimatedPosition = PVector.add(this.estimatedPosition, PVector.mult(this.estimatedVelocity, this.skeleton.dampingFactor*this.skeleton.scene.currentDeltaT)).mult(adjustedConfidenceParametersForresponseTradeoff[1])
                                            .add(PVector.add(this.parentJoint.estimatedPosition, PVector.mult(this.parentBone.currentEstimatedDirection, this.parentBone.estimatedLength)).mult(adjustedConfidenceParametersForresponseTradeoff[2]))
                                            .add(PVector.mult(this.measuredPosition, adjustedConfidenceParametersForresponseTradeoff[0]));
    }
    float distanceBetweenMeasuredPositionAndEstimatedPosition = PVector.sub(this.measuredPosition, newEstimatedPosition).mag();
    this.averageDistanceBetweenMeasuredPositionAndEstimatedPosition = lerp(this.averageDistanceBetweenMeasuredPositionAndEstimatedPosition, distanceBetweenMeasuredPositionAndEstimatedPosition, pow(confidenceParameters[0], this.skeleton.responseTradeoff));
    this.distanceBetweenMeasuredPositionAndEstimatedPositionStandardDeviation = lerp(this.distanceBetweenMeasuredPositionAndEstimatedPositionStandardDeviation, abs(distanceBetweenMeasuredPositionAndEstimatedPosition-this.averageDistanceBetweenMeasuredPositionAndEstimatedPosition), pow(confidenceParameters[0], this.skeleton.responseTradeoff));
    float measuredPositionConfidence = howCloseToTheMean(distanceBetweenMeasuredPositionAndEstimatedPosition, this.averageDistanceBetweenMeasuredPositionAndEstimatedPosition, this.distanceBetweenMeasuredPositionAndEstimatedPositionStandardDeviation); 
    
    float[] adjustedConfidenceParameters = adjustConfidenceParametersByAlphaMultiplier(confidenceParameters, measuredPositionConfidence);
    if(this.jointId!=1){ // If this joint has parentBone
      adjustedConfidenceParameters = adjustConfidenceParametersByAlphaMultiplier(adjustedConfidenceParameters, this.parentBone.measuredLengthConfidence);
    }
    
    if(jointId == 1){ // SpineMid
      newEstimatedPosition = PVector.lerp(PVector.add(this.estimatedPosition, PVector.mult(this.estimatedVelocity, this.skeleton.dampingFactor*this.skeleton.scene.currentDeltaT)), this.measuredPosition, adjustedConfidenceParameters[0]);
    } 
    else{ // Common Joints
      newEstimatedPosition = PVector.add(this.estimatedPosition, PVector.mult(this.estimatedVelocity, this.skeleton.dampingFactor*this.skeleton.scene.currentDeltaT)).mult(adjustedConfidenceParameters[1])
                                            .add(PVector.add(this.parentJoint.estimatedPosition, PVector.mult(this.parentBone.currentEstimatedDirection, this.parentBone.estimatedLength)).mult(adjustedConfidenceParameters[2]))
                                            .add(PVector.mult(this.measuredPosition, adjustedConfidenceParameters[0]));
    }
    PVector newEstimatedVelocity = PVector.sub(newEstimatedPosition, this.estimatedPosition).div(this.skeleton.scene.currentDeltaT);
    this.estimatedAcceleration = PVector.sub(newEstimatedVelocity, this.estimatedVelocity).div(this.skeleton.scene.currentDeltaT);
    this.estimatedVelocity = newEstimatedVelocity;
    this.estimatedPosition = newEstimatedPosition;
/*
    if(this.jointId == FOOT_LEFT){
      println("LeftFoot accel: "+this.estimatedAcceleration.mag());
      println("LeftFoot velocity: "+this.estimatedVelocity.mag());
    }*/
  }
  
/**
 * Iterates one step on the estimation of the current orientation.
 * First, it judges if the new measurement is an outlier or not, by calculating how close to the mean it is.
 * Then it uses this step size to calculate the new estimatedOrientation using the previousEstimatedOrientation and the current measurement. 
 * It uses quaternion extrapolation (qSlerp with step>1) to account for orientation velocity.
 * @param confidenceParameters (alpha, beta, gamma).
 */
  private void updateOrientation(float[] confidenceParameters){
    Quaternion predictedCurrentOrientation = qSlerp(this.previousEstimatedOrientation, this.estimatedOrientation, 1 + this.skeleton.dampingFactor*this.skeleton.scene.currentDeltaT/this.skeleton.scene.previousDeltaT);
    float measuredAngleBetweenMeasuredOrientationAndPredictedCurrentOrientation = angleBetweenQuaternions(this.measuredOrientation, predictedCurrentOrientation);
    this.averageAngleBetweenMeasuredOrientationAndPredictedCurrentOrientation = lerp(this.averageAngleBetweenMeasuredOrientationAndPredictedCurrentOrientation, measuredAngleBetweenMeasuredOrientationAndPredictedCurrentOrientation, pow(confidenceParameters[0], this.skeleton.responseTradeoff));
    this.angleBetweenMeasuredOrientationAndPredictedCurrentOrientationStandardDeviation = lerp(this.angleBetweenMeasuredOrientationAndPredictedCurrentOrientationStandardDeviation, abs(measuredAngleBetweenMeasuredOrientationAndPredictedCurrentOrientation-this.averageAngleBetweenMeasuredOrientationAndPredictedCurrentOrientation), pow(confidenceParameters[0], this.skeleton.responseTradeoff));
    float orientationStep = howCloseToTheMean(measuredAngleBetweenMeasuredOrientationAndPredictedCurrentOrientation, this.averageAngleBetweenMeasuredOrientationAndPredictedCurrentOrientation, this.angleBetweenMeasuredOrientationAndPredictedCurrentOrientationStandardDeviation); 
    Quaternion newEstimatedOrientation = qSlerp(predictedCurrentOrientation, this.measuredOrientation, orientationStep*confidenceParameters[0]);
    this.previousEstimatedOrientation = this.estimatedOrientation;
    this.estimatedOrientation = newEstimatedOrientation;
  }
  
/**
 * Adjust the confidence of the new measurements (alpha) of the joint based on its trackingState received from Kinect.
 * If inferred, confidence = confidence/3. 
 * If not tracked, confidence = confidence/9.
 * Then it proportionally redistributes that confidence removed from alpha to beta and gamma.
 * @param confidenceParameters array of: alpha, beta, gamma.
 * @param trackingState received from kinect.
 * @return adjustedConfidenceParameters (alpha, beta, gamma).
 */
  private float[] adjustConfidenceParametersByTrackingState(float[] confidenceParameters, int trackingState){
    float alphaAdjusted;
    if(trackingState == 2){ // If joint is tracked
      alphaAdjusted = confidenceParameters[0];
    }
    else if(trackingState == 1){ // If joint is inferred
      alphaAdjusted = confidenceParameters[0]/3;
    }
    else{ //(trackingState == 0) // If joint is not tracked
      alphaAdjusted = confidenceParameters[0]/9;
    }
    float betaAdjusted = confidenceParameters[1] + (confidenceParameters[0]-alphaAdjusted)*confidenceParameters[1]/(confidenceParameters[1]+confidenceParameters[2]);
    float gammaAdjusted = confidenceParameters[2] + (confidenceParameters[0]-alphaAdjusted)*confidenceParameters[2]/(confidenceParameters[1]+confidenceParameters[2]);
    float[] adjustedConfidenceParameters = {alphaAdjusted, betaAdjusted, gammaAdjusted};
    return adjustedConfidenceParameters;
  }
  
/**
 * Remove confidence of new measurements to get only the estimated position.
 * Then it proportionally redistributes that confidence removed from alpha to beta and gamma.
 * @param confidenceParameters array of: alpha, beta, gamma.
 * @return adjustedConfidenceParameters (alpha, beta, gamma).
 */
  private float[] adjustConfidenceParametersByResponseTradeoff(float[] confidenceParameters){
    float alphaAdjusted = 0;
    float betaAdjusted = confidenceParameters[1] + (confidenceParameters[0]-alphaAdjusted)*confidenceParameters[1]/(confidenceParameters[1]+confidenceParameters[2]);
    float gammaAdjusted = confidenceParameters[2] + (confidenceParameters[0]-alphaAdjusted)*confidenceParameters[2]/(confidenceParameters[1]+confidenceParameters[2]);
    float[] adjustedConfidenceParameters = {alphaAdjusted, betaAdjusted, gammaAdjusted};
    return adjustedConfidenceParameters;
  }

/**
 * Adjust the confidence of the new measurements (alpha) using a generic multiplier (between 0 and 1).
 * Then it proportionally redistributes that confidence removed from alpha to beta and gamma.
 * @param confidenceParameters array of: alpha, beta, gamma.
 * @param alphaMultiplier generic multiplier (between 0 and 1).
 * @return adjustedConfidenceParameters (alpha, beta, gamma).
 */
  private float[] adjustConfidenceParametersByAlphaMultiplier(float[] confidenceParameters, float alphaMultiplier){
    float alphaAdjusted = confidenceParameters[0]*alphaMultiplier;
    float betaAdjusted = confidenceParameters[1] + (confidenceParameters[0]-alphaAdjusted)*confidenceParameters[1]/(confidenceParameters[1]+confidenceParameters[2]);
    float gammaAdjusted = confidenceParameters[2] + (confidenceParameters[0]-alphaAdjusted)*confidenceParameters[2]/(confidenceParameters[1]+confidenceParameters[2]);
    float[] adjustedConfidenceParameters = {alphaAdjusted, betaAdjusted, gammaAdjusted};
    return adjustedConfidenceParameters;
  }  
  
/**
 * Calculate the directions of each axis (x, y, z) of the joint coordinate system.
 */
  private void calculateCoordinateSystemOrientation(){
    this.estimatedDirectionX = qMult(qMult(this.estimatedOrientation, new Quaternion(0, 1, 0, 0)), qConjugate(this.estimatedOrientation)).vector; 
    this.estimatedDirectionY = qMult(qMult(this.estimatedOrientation, new Quaternion(0, 0, 1, 0)), qConjugate(this.estimatedOrientation)).vector; 
    this.estimatedDirectionZ = qMult(qMult(this.estimatedOrientation, new Quaternion(0, 0, 0, 1)), qConjugate(this.estimatedOrientation)).vector; 
    this.measuredDirectionX = qMult(qMult(this.measuredOrientation, new Quaternion(0, 1, 0, 0)), qConjugate(this.measuredOrientation)).vector; 
    this.measuredDirectionY = qMult(qMult(this.measuredOrientation, new Quaternion(0, 0, 1, 0)), qConjugate(this.measuredOrientation)).vector; 
    this.measuredDirectionZ = qMult(qMult(this.measuredOrientation, new Quaternion(0, 0, 0, 1)), qConjugate(this.measuredOrientation)).vector; 
  }
  
/**
 * Drawing method.
 * @param drawEstimatedOrientation boolean indicating if the estimated orientation should be drawn.
 * @param drawMeasuredOrientation boolean indicating if the measured orientation should be drawn.
 */
  public void draw(boolean drawMeasuredOrientation, boolean drawEstimatedOrientation){
    this.drawPosition(this.skeleton.colorEstimated);
    if(!this.isEndJoint){
      this.drawOrientation(drawEstimatedOrientation,  drawMeasuredOrientation); // estimated, measured
    }
  }
  
/**
 * Draw a sphere on its position.
 * @param color.
 */
  public void drawPosition(color c){
    strokeWeight(1);
    sphereDetail(6);
    if(this.trackingState == 2){ // normal sphere if tracked
      stroke(c);
      fill(c);
    } else if(this.trackingState == 1){ // black contour if inferred
      stroke(color(0, 0, 0, 170)); 
      fill(c);
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
  
    
/**
 * Draw the joint coordinate system on screen.
 * @param drawEstimated boolean indicating if the estimated orientation should be drawn.
 * @param drawMeasured boolean indicating if the measured orientation should be drawn.
 */
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
