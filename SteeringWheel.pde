/**
 * Abstraction of a steering wheel being grabbed by the user's hands. While grabbed (closed hands), it can be steered in 3D.
 * rollStep, pitchStep and yawStep should be used to increment the rotation each draw loop.
 */
class SteeringWheel{
  public Skeleton skeleton;
  public boolean isGrabbed = false;
  public PVector position = new PVector(0, 0, 1.5);
  public PVector positionPercentageOfRoom = new PVector(0, 0, 0);
  public float rollAngle;
  public float rollSize;
  public float rollStep;
  public float pitchAngle;
  public float pitchSize;
  public float pitchStep;
  public float yawAngle;
  public float yawSize;
  public float yawStep;
  public PVector vectorConnectingHands; // From left to right
  
  public SteeringWheel(Skeleton skeleton){
    this.skeleton = skeleton;
  }
  
  private void update(){
    //if(this.skeleton.estimatedHandRadius[0]<0.5 && this.skeleton.estimatedHandRadius[1]<0.5) this.isGrabbed = true; else this.isGrabbed = false;
    if(this.skeleton.estimatedHandRadius[0] + this.skeleton.estimatedHandRadius[1] < 0.75) this.isGrabbed = true; else this.isGrabbed = false;
    if(isGrabbed){
      this.vectorConnectingHands = PVector.sub(this.skeleton.joints[HAND_RIGHT].estimatedPosition, this.skeleton.joints[HAND_LEFT].estimatedPosition);
      if(this.skeleton.scene.floor.isCalibrated){
        this.position = this.skeleton.scene.floor.toFloorCoordinateSystem(PVector.lerp(this.skeleton.joints[HAND_LEFT].estimatedPosition, this.skeleton.joints[HAND_RIGHT].estimatedPosition, 0.5));
        this.positionPercentageOfRoom.x = this.position.x/(this.skeleton.scene.floor.dimensions.x/2);
        this.positionPercentageOfRoom.y = this.position.y/(this.skeleton.scene.floor.dimensions.y/2);
        this.positionPercentageOfRoom.z = this.position.z/(this.skeleton.scene.floor.dimensions.z/2);
        this.rollAngle = atan(PVector.dot(this.vectorConnectingHands, this.skeleton.scene.floor.basisVectorY)/PVector.dot(this.vectorConnectingHands, this.skeleton.scene.floor.basisVectorX));
        this.rollSize = abs(sqrt(sq(PVector.dot(this.vectorConnectingHands, this.skeleton.scene.floor.basisVectorX))+sq(PVector.dot(this.vectorConnectingHands, this.skeleton.scene.floor.basisVectorY))));
        this.pitchAngle = atan(PVector.dot(this.vectorConnectingHands, this.skeleton.scene.floor.basisVectorZ)/PVector.dot(this.vectorConnectingHands, this.skeleton.scene.floor.basisVectorY));
        this.pitchSize = abs(PVector.dot(this.vectorConnectingHands, this.skeleton.scene.floor.basisVectorY));
        this.yawAngle = atan(PVector.dot(this.vectorConnectingHands, this.skeleton.scene.floor.basisVectorZ)/PVector.dot(this.vectorConnectingHands, this.skeleton.scene.floor.basisVectorX));
        this.yawSize = abs(PVector.dot(this.vectorConnectingHands, this.skeleton.scene.floor.basisVectorX));
      } else {
        this.position = PVector.lerp(this.skeleton.joints[HAND_LEFT].estimatedPosition, this.skeleton.joints[HAND_RIGHT].estimatedPosition, 0.5);
        this.rollAngle = atan(this.vectorConnectingHands.y/this.vectorConnectingHands.x);
        this.rollSize = abs(sqrt(sq(this.vectorConnectingHands.x)+sq(this.vectorConnectingHands.y)));
        this.pitchAngle = atan(this.vectorConnectingHands.z/this.vectorConnectingHands.y);
        this.pitchSize = abs(this.vectorConnectingHands.y);
        this.yawAngle = atan(this.vectorConnectingHands.z/this.vectorConnectingHands.x);
        this.yawSize = abs(this.vectorConnectingHands.x);
      }
    } else {
      this.position.y = 0;
      this.rollSize = 0;
      this.pitchSize = 0;
      this.yawSize = 0;
    }
    this.rollStep = this.rollAngle*this.rollSize/10;
    this.pitchStep = this.pitchAngle*this.pitchSize/10;
    this.yawStep = this.yawAngle*this.yawSize/10;
  }
}
