class Features{
  public float[] legAngle = new float[2]; // {Left, Right}
  public float[] shoulderAngle = new float[2]; // {Left, Right}
  public float[] elbowAngle = new float[2]; // {Left, Right}
  public float distanceBetweenHands;
  private Skeleton skeleton;
  
  public Features(Skeleton skeleton){
    this.skeleton = skeleton;
    this.updateFeatures();
  }
  
  public void updateFeatures(){
    this.legAngle[0] = PVector.angleBetween(PVector.sub(skeleton.joints[13].estimatedPosition, skeleton.joints[14].estimatedPosition), PVector.sub(skeleton.joints[12].estimatedPosition, skeleton.joints[13].estimatedPosition));
    this.legAngle[1] = PVector.angleBetween(PVector.sub(skeleton.joints[17].estimatedPosition, skeleton.joints[18].estimatedPosition), PVector.sub(skeleton.joints[16].estimatedPosition, skeleton.joints[17].estimatedPosition));
    this.shoulderAngle[0] = PVector.angleBetween(PVector.sub(skeleton.joints[1].estimatedPosition, skeleton.joints[20].estimatedPosition), PVector.sub(skeleton.joints[5].estimatedPosition, skeleton.joints[4].estimatedPosition));
    this.shoulderAngle[1] = PVector.angleBetween(PVector.sub(skeleton.joints[1].estimatedPosition, skeleton.joints[20].estimatedPosition), PVector.sub(skeleton.joints[9].estimatedPosition, skeleton.joints[8].estimatedPosition));
    this.elbowAngle[0] = PVector.angleBetween(PVector.sub(skeleton.joints[4].estimatedPosition, skeleton.joints[5].estimatedPosition), PVector.sub(skeleton.joints[6].estimatedPosition, skeleton.joints[5].estimatedPosition));
    this.elbowAngle[1] = PVector.angleBetween(PVector.sub(skeleton.joints[8].estimatedPosition, skeleton.joints[9].estimatedPosition), PVector.sub(skeleton.joints[10].estimatedPosition, skeleton.joints[9].estimatedPosition));
    this.distanceBetweenHands = PVector.sub(skeleton.joints[7].estimatedPosition, skeleton.joints[11].estimatedPosition).mag();
  }
}
