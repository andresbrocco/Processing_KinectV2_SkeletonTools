/**
 * Pollock: Trigger Affordance that quantifies the "throwing" movement of each hand.
 */
public class Pollock{
  private Joint handJoint;
  private Joint shoulderJoint;
  private Joint headJoint;
  private Joint spineShoulder;
  private PVector shoulderToHandVelocity = new PVector(0, 0, 0);
  private float shoulderToHandSpeed = 0;
  private float speedThreshold = 0.75;
  private boolean isAboveSpeedThreshold = false;
  private int activationMillis;
  private PVector headPositionWhenActivated;
  private PVector handPositionWhenActivated;
  private int direction = 0;
  
  Pollock(Skeleton skeleton, String whichHand){
    if(whichHand == "left"){
      this.handJoint = skeleton.joints[HAND_LEFT];
      this.shoulderJoint = skeleton.joints[SHOULDER_LEFT];
    }else if(whichHand == "right"){
      this.handJoint = skeleton.joints[HAND_RIGHT];
      this.shoulderJoint = skeleton.joints[SHOULDER_RIGHT];
    }
    this.headJoint = skeleton.joints[HEAD];
    this.spineShoulder = skeleton.joints[SPINE_SHOULDER];
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
