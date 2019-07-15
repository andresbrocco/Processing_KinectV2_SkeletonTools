/**
 * Check if the joint triggered the pollock threshold. If so, find which sextant it is pointing.
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
 
public class Pollock{
  private Joint joint;
  private int direction = 0;
  
  Pollock(Joint joint){
    this.joint = joint;
  }
  
  private void update(){
    float jointSpeed = this.joint.estimatedVelocity.mag();
    if(this.joint.jointId == HAND_LEFT){
      if(jointSpeed > 1.5){
        this.direction = 1;
        //println("jointVelocity: "+jointSpeed);
      } else {
        this.direction = 0;
      }
    }
  }
  
  private void draw(){
    if(this.direction != 0){
      pushMatrix();
      strokeWeight(5);
      translate(reScaleX(this.joint.estimatedPosition.x),
                reScaleY(this.joint.estimatedPosition.y),
                reScaleZ(this.joint.estimatedPosition.z));
      PVector arrow = PVector.mult(this.joint.estimatedVelocity, 1);
      line(0, 0, 0, reScaleX(arrow.x), reScaleY(arrow.y), reScaleZ(arrow.z));
      popMatrix();
    }
  }
  
  /*
  private void updatePollock(){
    float threshold = 10;
    for(int j = 7; j<12; j=j+4){
      int azimuth = 0;
      int pollock = 0;
      PVector jointAcceleration = this.skeleton.joints[j].estimatedAcceleration;
      if(jointAcceleration.mag() > threshold){
        PVector jointAccelerationRelativeToSpineMidCSys = new PVector(PVector.dot(jointAcceleration, this.skeleton.joints[SPINE_MID].estimatedDirectionX), 
                                                                      PVector.dot(jointAcceleration, this.skeleton.joints[SPINE_MID].estimatedDirectionY), 
                                                                      PVector.dot(jointAcceleration, this.skeleton.joints[SPINE_MID].estimatedDirectionZ));
        float thresholdDotProduct = sqrt(2)/2;
        PVector jointAccelerationProjectionInAzimuthPlaneNormalized = (new PVector(jointAccelerationRelativeToSpineMidCSys.x, 0, jointAccelerationRelativeToSpineMidCSys.z)).normalize();
             if(jointAccelerationProjectionInAzimuthPlaneNormalized.x  >  thresholdDotProduct) azimuth = 1; // Azimuth: Left
        else if(jointAccelerationProjectionInAzimuthPlaneNormalized.x  <  thresholdDotProduct) azimuth = 3; // Azimuth: Right
        else if(jointAccelerationProjectionInAzimuthPlaneNormalized.z  >  thresholdDotProduct) azimuth = 2; // Azimuth: Front
        else if(jointAccelerationProjectionInAzimuthPlaneNormalized.z  <= thresholdDotProduct) azimuth = 4; // Azimuth: Back
        
        PVector jointAccelerationProjectionInAltitudePlaneNormalized = (new PVector(0, jointAccelerationRelativeToSpineMidCSys.y, jointAccelerationRelativeToSpineMidCSys.z)).normalize();
        if(jointAccelerationProjectionInAltitudePlaneNormalized.y >  thresholdDotProduct){ // Altitude: Up
          pollock = pollock + 10; 
        }
        else if(jointAccelerationProjectionInAltitudePlaneNormalized.y <  thresholdDotProduct) pollock = pollock + 30; // Altitude: Down
        else if(jointAccelerationProjectionInAltitudePlaneNormalized.z >  thresholdDotProduct) pollock = pollock + 20; // Altitude: Front
        else if(jointAccelerationProjectionInAltitudePlaneNormalized.z <= thresholdDotProduct) pollock = pollock + 40; // Altitude: Back
        
        
        println("pollock " + pollock + " from joint: " + j + " with acelleration magnitude: " + jointAcceleration.mag());
        this.drawPollock(pollock);
      }
      this.skeleton.joints[j].pollock = pollock;
    }
  }
  */
}
