public class Skeleton{
  private Scene scene;
  private int indexColor; // Which body is it (color = - M*255*256^2 - C*255*256 - Y*255 -1)
  private int[] skeletonColorRGB = new int[3]; // in RGB
  private int appearedLastInFrame = 0; // counter to keep track if skeleton is dead or not.
  private float alpha = 0.33; // alpha = confidence of new measurement
  private float beta = 0.33; // beta = confidence of estimated position based on previous position and velocity
  private float gamma = 1 - this.alpha - this.beta; // gamma = confidence of estimated position based on parentBone orientation and length
  private float[] confidenceParameters = {alpha, beta, gamma};
  private float dampingFactor = 0.707; // 1 is not damped. 0 is fully damped.
  private boolean isTracked = false; // dumb information, because is always true.
  private int[] measuredHandStates = {0, 0}; // Left, Right
  private float[] measuredHandRadius = {0, 0}; // goes from 0 to 1, indicating how opened the hand is.
  private float[] estimatedHandRadius = {0.5, 0.5}; // goes from 0 to 1, indicating how opened the hand is.
  private Joint[] joints = new Joint[25];
  private Bone[] bones = new Bone[24];
  public Features features;
  private int[][] skeletonConnections = {// {boneId, parentJointId, childJointId}
                                {0 , SPINE_MID, SPINE_BASE}, 
                                {1 , SPINE_MID, SPINE_SHOULDER}, 
                                {2 , SPINE_SHOULDER, NECK}, 
                                {3 , NECK , HEAD},
                                {4 , SPINE_SHOULDER, SHOULDER_LEFT}, 
                                {5 , SHOULDER_LEFT , ELBOW_LEFT},
                                {6 , ELBOW_LEFT, WRIST_LEFT }, 
                                {7 , WRIST_LEFT, HAND_LEFT},
                                {8 , SPINE_SHOULDER, SHOULDER_RIGHT}, 
                                {9 , SHOULDER_RIGHT, ELBOW_RIGHT}, 
                                {10, ELBOW_RIGHT, WRIST_RIGHT}, 
                                {11, WRIST_RIGHT, HAND_RIGHT}, 
                                {12, SPINE_BASE , HIP_LEFT}, 
                                {13, HIP_LEFT, KNEE_LEFT}, 
                                {14, KNEE_LEFT, ANKLE_LEFT},
                                {15, ANKLE_LEFT, FOOT_LEFT},
                                {16, SPINE_BASE, HIP_RIGHT},
                                {17, HIP_RIGHT, KNEE_RIGHT},
                                {18, KNEE_RIGHT, ANKLE_RIGHT},
                                {19, ANKLE_RIGHT, FOOT_RIGHT},
                                {20, HAND_LEFT , HAND_TIP_LEFT},
                                {21, WRIST_LEFT , THUMB_LEFT}, 
                                {22, HAND_RIGHT, HAND_TIP_RIGHT}, 
                                {23, WRIST_RIGHT, THUMB_RIGHT}};
                      
  public Skeleton(KSkeleton kSkeleton, Scene scene){
    this.scene = scene;
    this.indexColor = kSkeleton.getIndexColor();
    this.skeletonColorRGB = this.convertIndexColorToRGB(this.indexColor);
    this.isTracked = kSkeleton.isTracked();
    this.measuredHandStates[0] = kSkeleton.getLeftHandState();
    this.measuredHandStates[1] = kSkeleton.getRightHandState();
    this.measureHandRadius();
    KJoint[] kJoints = kSkeleton.getJoints();
    for(int j=0; j<25; j++){ // Create all Joints
      this.joints[j] = new Joint(j, kJoints[j], this);
    }
    for(int b=0; b<24; b++){ // Create all bones, tell the childJoint who is its parentJoint and parentBone
    //The SpineMid Joint won't have a parentJoint, neither a parentBone. 
      this.bones[b] = new Bone(b, this.joints[skeletonConnections[b][1]], this.joints[skeletonConnections[b][2]]);
      this.joints[skeletonConnections[b][2]].setParentJoint(this.joints[skeletonConnections[b][1]]);
      this.joints[skeletonConnections[b][2]].setParentBone(this.bones[b]);
    }
    for(int b=0; b<24; b++){
      this.joints[skeletonConnections[b][1]].addChildBone(this.bones[skeletonConnections[b][0]]);
    }
    this.appearedLastInFrame = frameCount;
    this.features = new Features(this);
  }
  
  public void update(KSkeleton kSkeleton, float currentDeltaT, float previousDeltaT){  //<>//
    this.isTracked = kSkeleton.isTracked(); 
    this.measuredHandStates[0] = kSkeleton.getLeftHandState();
    this.measuredHandStates[1] = kSkeleton.getRightHandState();
    this.measureHandRadius();
    KJoint[] kJoints = kSkeleton.getJoints();
    for (int j=0; j<25; j++){
      joints[j].receiveNewMeasurements(kJoints[j]);
    }
    this.smoothSkeleton(currentDeltaT, previousDeltaT, dampingFactor);
    this.appearedLastInFrame = frameCount;
    this.features.updateFeatures();
  }
  
  private int[] convertIndexColorToRGB(int indexColor){ // The color shown at the kinect studio interface is not the color that it sends. I think....
    int C = (-indexColor-1>>16);
    int M = ((-indexColor-1-(C<<16))>>8);
    int Y = ((-indexColor-1-(C<<16)-(M<<8))>>0);
    int[] rgb = new int[3];
    //println("CMY: "+C +" "+ M + " "+ Y);
    rgb[0] = 255-C; // R
    rgb[1] = 255-M; // G
    rgb[2] = 255-Y; // B
    //println("RGB: "+rgb[0] +" "+ rgb[1] + " "+ rgb[2]);
    return rgb;
  }
  
  private void smoothSkeleton(float currentDeltaT, float previousDeltaT, float dampingFactor){  //<>// //<>// //<>//
    this.joints[1].calculateEstimates(this.confidenceParameters, currentDeltaT, previousDeltaT, dampingFactor); // trigger the chain reaction by calling the SpineMid to be calculated.
    this.smoothHandRadius(this.alpha);
  }
  
  private void smoothHandRadius(float alpha){
    for(int h=0; h<2; h++){
      if(this.measuredHandRadius[h] == -1){ // if hand is not tracked.
        break;
      } else{
        this.estimatedHandRadius[h] = this.estimatedHandRadius[h]*(1-alpha) + this.measuredHandRadius[h]*alpha;
      }
    }
  }
  
  private void measureHandRadius(){ // -1 if hand is not tracked or unknown.
    for(int h=0; h<2; h++){
      if(this.measuredHandStates[h] == 4){ // HAND_LASSO
        this.measuredHandRadius[h] = 0.5;
      } else if(this.measuredHandStates[h] == 3){ // HAND_CLOSED
        this.measuredHandRadius[h] = 0;
      } else if(this.measuredHandStates[h] == 2){ // HAND_OPEN
        this.measuredHandRadius[h] = 1;
      } else if(this.measuredHandStates[h] == 1){ // HAND_NOT_TRACKED
        this.measuredHandRadius[h] = -1;
      } else {// if(this.measuredHandStates[h] == 0) // HAND_UNKNOWN
        this.measuredHandRadius[h] = 0.5;
      }
    }
  }
  
  public void draw(boolean measuredSkeleton, boolean jointOrientation, boolean boneRelativeOrientation,  boolean handRadius, boolean handStates){
    color colorEstimated = color(this.skeletonColorRGB[0], this.skeletonColorRGB[1], this.skeletonColorRGB[2], 170);
    color colorMeasured = color(this.skeletonColorRGB[0], this.skeletonColorRGB[1], this.skeletonColorRGB[2], 85);
    for(Bone bone:this.bones){
      bone.draw(colorEstimated, colorMeasured, measuredSkeleton, boneRelativeOrientation);
    }
    for(Joint joint:this.joints){
      joint.draw(colorEstimated, measuredSkeleton, jointOrientation);
    }
    if(handRadius){
      this.drawHandRadius();
    }
    if(handStates){
      this.drawHandStates();
    }
  }
  
  public void drawHandRadius(){
    int maxSphereRadius = 25;
    stroke(color(0, 0, 0, 85));
    for(int h=0; h<2; h++){
      float sphereRadius = this.estimatedHandRadius[h]*maxSphereRadius;
      if(sphereRadius < 1){
        sphereRadius = 1;
      }
      strokeWeight(maxSphereRadius/sphereRadius);
      noFill();
      sphereDetail((int)(sphereRadius*2/3)+3);
      pushMatrix();
      if(h==0){ // left hand
        translate(reScaleX(this.joints[HAND_LEFT].estimatedPosition.x),
                  reScaleY(this.joints[HAND_LEFT].estimatedPosition.y),
                  reScaleZ(this.joints[HAND_LEFT].estimatedPosition.z));
      } else{ // right hand
        translate(reScaleX(this.joints[HAND_RIGHT].estimatedPosition.x),
                  reScaleY(this.joints[HAND_RIGHT].estimatedPosition.y),
                  reScaleZ(this.joints[HAND_RIGHT].estimatedPosition.z));
      }
      sphere(sphereRadius);
      popMatrix();
    }
  }
  
  public void drawHandStates(){
    int sphereRadius = 25;
    color handStateStrokeColor = color(0, 0, 0, 85);
    noStroke();
    for(int h=0; h<2; h++){
      if(this.measuredHandStates[h] == 4){ // HAND_LASSO
        sphereRadius = 12;
        stroke(handStateStrokeColor);
        strokeWeight(25/sphereRadius);
      } else if(this.measuredHandStates[h] == 3){ // HAND_CLOSED
        sphereRadius = 6;
        stroke(handStateStrokeColor);
        strokeWeight(25/sphereRadius);
      } else if(this.measuredHandStates[h] == 2){ // HAND_OPEN
        sphereRadius = 25;
        stroke(handStateStrokeColor);
        strokeWeight(25/sphereRadius);
      } else if(this.measuredHandStates[h] == 1){ // HAND_NOT_TRACKED
      } else {// if(this.handStates[h] == 0) // HAND_UNKNOWN
      }
      noFill();
      sphereDetail((int)(sphereRadius*2/3));
      pushMatrix();
      
      if(h==0){ // left hand
        translate(reScaleX(this.joints[HAND_LEFT].estimatedPosition.x),
                  reScaleY(this.joints[HAND_LEFT].estimatedPosition.y),
                  reScaleZ(this.joints[HAND_LEFT].estimatedPosition.z));
      } else{ // right hand
        translate(reScaleX(this.joints[HAND_RIGHT].estimatedPosition.x),
                  reScaleY(this.joints[HAND_RIGHT].estimatedPosition.y),
                  reScaleZ(this.joints[HAND_RIGHT].estimatedPosition.z));
      }
      sphere(sphereRadius);
      popMatrix();
    }
  }
}
