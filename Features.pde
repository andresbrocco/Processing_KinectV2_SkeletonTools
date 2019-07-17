/**
 * Each skeleton has a feature object. All indirect features must be calculated here. This is where you have fun!
 */
class Features{
  private Skeleton skeleton;
  public PVector leftHandPositionLocal;
  public Quaternion leftHandOrientationLocal;
  
  public Features(Skeleton skeleton){
    this.skeleton = skeleton;
    this.update();
  }

/**
 * Updates all the "high-level" features.
 */
  public void update(){ // substitute indexes by respective joint name from "skeletonConstants" tab.
    /*this.steeringWheel.update();
    this.updateMomentum();
    this.updateHeadInclination();
    this.updateShoulderTension();
    this.updatePollock();
    
    // To get orientations and positions relative to the floor coordinate system, use the floor method:
    this.leftHandPositionLocal = this.skeleton.scene.floor.toFloorCoordinateSystem(this.skeleton.joints[HAND_LEFT].estimatedPosition); 
    this.leftHandOrientationLocal = this.skeleton.scene.floor.toFloorCoordinateSystem(this.skeleton.joints[HAND_LEFT].estimatedOrientation); // not tested yet;
    */
  }
  
  
}
