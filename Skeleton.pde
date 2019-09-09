/**  //<>//
 * A skeleton contains all the information related to a specific body. It also contains the algorithms to smooth its movement and draw itself on screen.
 */
public class Skeleton{
  // Core attributes:
  private Scene scene;
  private int indexColor; // Which body is it (color = - M*255*256^2 - C*255*256 - Y*255 -1)
  private int[] skeletonColorRGB = new int[3]; // in RGB
  private color colorEstimated;
  private color colorMeasured;
  private int appearedLastInFrame = 0; // counter to keep track if skeleton is dead or not.
  private float responseTradeoff = 0.2; // Tradeoff between speed and smoothness: close to 0 gives faster responses but more noise. Larger values give lazy skeleton. 
  private float alpha = 0.5; // alpha = confidence of new measurement
  private float beta = 0.25; // beta = confidence of estimated position based on previous position and velocity
  private float gamma = 1 - this.alpha - this.beta; // gamma = confidence of estimated position based on parentBone orientation and length
  private float[] confidenceParameters = {alpha, beta, gamma};
  private float dampingFactor = 0.707; // 1 is not damped. 0 is fully damped.
  private boolean isTracked = false; // dumb information, because is always true.
  private int[] measuredHandStates = {0, 0}; // Left, Right
  private float[] measuredHandRadius = {0, 0}; // goes from 0 to 1, indicating how opened the hand is.
  private float[] estimatedHandRadius = {0.5, 0.5}; // goes from 0 to 1, indicating how opened the hand is.
  private Joint[] joints = new Joint[25];
  private Bone[] bones = new Bone[24];
  public PrintWriter savingOutput;
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
  // Skeleton Features:
  public float bodySize; // Average length of all bones. Ideally, it should be normalized so that an average person would have bodySize = 1... This normalization have yet to be implemented.
  public float headInclination = 0; // head inclination relative to Z axis, in radians
  public float shoulderTension = 0; // SHOULDER height relative to SPINESHOULDER
  public SteeringWheel steeringWheel = new SteeringWheel(this);
  public float distanceBetweenHands;
  public PVector centerOfMass;
  public float centerOfMassHeightAdjusted; // Center of Mass height adjusted for body size. 
  public float dispersion; // variance of position of joints, adjusted by body size. (sum of distances to the center of gravity). Ranges from ~1.5 to ~3.
  public Pollock leftHandPollock;
  public Pollock rightHandPollock;
  public RondDuBras leftHandRondDuBras;
  public RondDuBras rightHandRondDuBras;
  public Momentum momentum = new Momentum(this);
                      
  public Skeleton(KSkeleton kSkeleton, Scene scene){
    this.scene = scene;
    this.indexColor = kSkeleton.getIndexColor();
    this.skeletonColorRGB = this.convertIndexColorToRGB(this.indexColor);
    this.colorEstimated = color(this.skeletonColorRGB[0], this.skeletonColorRGB[1], this.skeletonColorRGB[2], 170);
    this.colorMeasured = color(this.skeletonColorRGB[0], this.skeletonColorRGB[1], this.skeletonColorRGB[2], 85);
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
      this.bones[b] = new Bone(this, b, this.joints[skeletonConnections[b][1]], this.joints[skeletonConnections[b][2]]);
      this.joints[skeletonConnections[b][2]].setParentJoint(this.joints[skeletonConnections[b][1]]);
      this.joints[skeletonConnections[b][2]].setParentBone(this.bones[b]);
    }
    for(int b=0; b<24; b++){
      this.joints[skeletonConnections[b][1]].addChildBone(this.bones[skeletonConnections[b][0]]);
    }
    this.leftHandPollock = new Pollock(this, "LEFT");
    this.rightHandPollock = new Pollock(this, "RIGHT");
    this.leftHandRondDuBras = new RondDuBras(this, "LEFT");
    this.rightHandRondDuBras = new RondDuBras(this, "RIGHT");
    if(this.scene.saveSession) {
      this.savingOutput = createWriter("savedSessions/"+this.scene.sessionName+"/skeleton"+this.scene.numberOfSkeletons+".txt");
      this.savingOutput.print("frameCount bodySize centerOfMassHeightAdjusted dispersion leftPollock rightPollock leftRondDuBras rightRondDuBras momentumFluid momentumHarsh momentumTotal handRadiusLeft handRadiusRight ");
      for(int j=0; j<25; j++){
        this.savingOutput.print("joint"+j+"PositionX joint"+j+"PositionY joint"+j+"PositionZ joint"+j+"OrientationW joint"+j+"OrientationX joint"+j+"OrientationY joint"+j+"OrientationZ ");
      }
      this.savingOutput.println("");
      this.savingOutput.flush();
    }
    this.appearedLastInFrame = frameCount;
  }
  
/**
 * Receives new raw skeleton data from kinect, smooth its movement and updates its features. 
 * @param kSkeleton raw skeleton data from kinect.
 */
  public void update(KSkeleton kSkeleton){  
    this.isTracked = kSkeleton.isTracked(); 
    this.measuredHandStates[0] = kSkeleton.getLeftHandState();
    this.measuredHandStates[1] = kSkeleton.getRightHandState();
    this.measureHandRadius();
    KJoint[] kJoints = kSkeleton.getJoints();
    for (int j=0; j<25; j++){
      joints[j].receiveNewMeasurements(kJoints[j]);
    }
    this.smoothSkeleton();
    this.calculateBodySize();
    this.calculateCenterOfMass();
    this.calculateDispersion();
    this.leftHandPollock.update();
    this.rightHandPollock.update();
    this.leftHandRondDuBras.update();
    this.rightHandRondDuBras.update();
    this.momentum.update();
    this.appearedLastInFrame = frameCount;
    if(this.scene.saveSession && frameCount%this.scene.frameRate_==0) this.save();
  }
  
  private void save(){
    this.savingOutput.print(frameCount+" ");
    this.savingOutput.print(this.bodySize+" ");
    this.savingOutput.print(this.centerOfMassHeightAdjusted+" ");
    this.savingOutput.print(this.dispersion+" ");
    this.savingOutput.print(this.leftHandPollock.activationDirectionCode+" ");
    this.savingOutput.print(this.rightHandPollock.activationDirectionCode+" ");
    this.savingOutput.print(this.leftHandRondDuBras.activatedDirectionCode+" ");
    this.savingOutput.print(this.rightHandRondDuBras.activatedDirectionCode+" ");
    this.savingOutput.print(this.momentum.averageFluid+" ");
    this.savingOutput.print(this.momentum.averageHarsh+" ");
    this.savingOutput.print(this.momentum.averageTotal+" ");
    this.savingOutput.print(this.estimatedHandRadius[0]+" ");
    this.savingOutput.print(this.estimatedHandRadius[1]+" ");
    for(int j=0; j<25; j++){
      this.savingOutput.print(this.joints[j].estimatedPosition.x+" "+
                              this.joints[j].estimatedPosition.y+" "+
                              this.joints[j].estimatedPosition.z+" "+
                              this.joints[j].estimatedOrientation.real+" "+
                              this.joints[j].estimatedOrientation.vector.x+" "+
                              this.joints[j].estimatedOrientation.vector.y+" "+
                              this.joints[j].estimatedOrientation.vector.z+" ");
    }
    this.savingOutput.println("");
    this.savingOutput.flush();
    //this.savingOutput.close();
  }
  
/**
 * Average of lenghts of all bones. This value can be used to normalize feature parameters to adjust for different body sizes.
 */
  void calculateBodySize(){
    this.bodySize = 0;
    for(Bone bone:this.bones){
      this.bodySize += bone.estimatedLength/24;
    }
  }
  
/**
 * Center of Mass: average position of all joints.
 */
  void calculateCenterOfMass(){
    this.centerOfMass = new PVector(0, 0, 0);
    for(int j=0; j<25; j++){ 
      //this.centerOfMass = this.centerOfMass.lerp(this.joints[j].estimatedPosition, 1/(j+1));
      this.centerOfMass.add(PVector.div(this.joints[j].estimatedPosition, 25));
    }
    this.centerOfMassHeightAdjusted = this.scene.floor.toFloorCoordinateSystem(this.centerOfMass).y/this.bodySize;
    //println("this.centerOfMassHeightAdjusted: "+this.centerOfMassHeightAdjusted);
  }
  
/**
 * Variance of position of joints, adjusted by bodySize. (average of distances to the center of gravity). Ranges from ~1.5 to ~ 3.
 */
  void calculateDispersion(){
    this.dispersion = 0;
    for(int j=0; j<25; j++){ 
      this.dispersion += PVector.sub(this.joints[j].estimatedPosition, this.centerOfMass).mag()/25;
    }
    this.dispersion /= this.bodySize;
    //println("this.dispersion: "+this.dispersion);
  }
  
/**
 * The Kinect sends a color index in a format somewhat similar to "color = - M*255*256^2 - C*255*256 - Y*255 -1".
 * This method converts it back to rgb space.
 * @param indexColor color sent from Kinect.
 * @return rgb color in RGB space.
 */
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
  
/**
 * Method to smooth the skeleton data.
 * The human body Center of Mass is right on the "spineMid" joint, so the smoothing altorithm starts updating from this joint.
 * This triggers a chain reaction, because each joint calls its next bone to be updated, and each bone calls its next joint, until the body extremes are reached.
 */
  private void smoothSkeleton(){
    this.joints[1].update(this.confidenceParameters); // trigger the chain reaction by calling the SpineMid to be calculated.
    this.smoothHandRadius(this.alpha);
  }
  
/**
 * Method to smooth the handRadius.
 * @param alpha interpolation step.
 */
  private void smoothHandRadius(float alpha){
    for(int h=0; h<2; h++){
      if(this.measuredHandRadius[h] == -1){ // if hand is not tracked.
        break;
      } else{
        this.estimatedHandRadius[h] = lerp(this.estimatedHandRadius[h], this.measuredHandRadius[h], alpha/4);
      }
    }
  }
  
/**
 * Method to convert from Kinect handState to the handRadius. The handRadius is 0 if closed, 0.5 if lasso or unknown, 1 if opened.
 * If it is not tracked, handRadius is -1 to indicate it must be discarded.
 */
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
  
/**
 * Method to draw the skeleton on screen.
 * @param drawMeasured indicates if measured bones should be drawn.
 * @param drawJointOrientation indicates if joint orientations should be drawn.
 * @param drawBoneRelativeOrientation indicates if bone relative orientations should be drawn.
 * @param drawHandRadius indicates if hand radius' should be drawn.
 * @param drawHandStates indicates if raw hand states should be drawn.
 */
  public void draw(boolean drawMeasured, boolean drawJointOrientation, boolean drawBoneRelativeOrientation, boolean drawHandRadius, 
                   boolean drawHandStates, boolean drawPollock, boolean drawRondDuBras, boolean drawMomentum, boolean drawCenterOfMass){
    for(Bone bone:this.bones)    bone.draw(drawMeasured, drawBoneRelativeOrientation);
    for(Joint joint:this.joints) joint.draw(drawMeasured, drawJointOrientation);
    if(drawHandRadius)           this.drawHandRadius();
    if(drawHandStates)           this.drawHandStates();
    if(drawPollock){
      this.leftHandPollock.draw(true, true, true);
      this.rightHandPollock.draw(true, true, true);
    }
    if(drawRondDuBras){
      this.leftHandRondDuBras.draw(true, true);
      this.rightHandRondDuBras.draw(true, true);
    }
    if(drawMomentum) this.momentum.draw();
    if(drawCenterOfMass) this.drawCenterOfMass(20);
    //this.drawSteeringWheel();
  }
  
/**
 * For testing only, shall be deprecated
 */
  private void drawSteeringWheel(){
    PVector vertex1 = PVector.mult(new PVector(cos(this.steeringWheel.yawAngle), 0, sin(this.steeringWheel.yawAngle)), 100*this.steeringWheel.yawSize);
    PVector vertex2 = PVector.mult(new PVector(-cos(this.steeringWheel.yawAngle), 0, -sin(this.steeringWheel.yawAngle)), 100*this.steeringWheel.yawSize);
    PVector vertex3 = PVector.mult(new PVector(0, cos(this.steeringWheel.pitchAngle), sin(this.steeringWheel.pitchAngle)), 100*this.steeringWheel.pitchSize);
    PVector vertex4 = PVector.mult(new PVector(0, -cos(this.steeringWheel.pitchAngle), -sin(this.steeringWheel.pitchAngle)), 100*this.steeringWheel.pitchSize);
    PVector vertex5 = PVector.mult(new PVector(cos(this.steeringWheel.rollAngle), sin(this.steeringWheel.rollAngle), 0), 100*this.steeringWheel.rollSize);
    PVector vertex6 = PVector.mult(new PVector(-cos(this.steeringWheel.rollAngle), -sin(this.steeringWheel.rollAngle), 0), 100*this.steeringWheel.rollSize);
    strokeWeight(5);
    stroke(color(128, 67, 23));
    line(vertex1.x, vertex1.y, vertex1.z, vertex2.x, vertex2.y, vertex2.z);
    stroke(color(67, 23, 128));
    line(vertex3.x, vertex3.y, vertex3.z, vertex4.x, vertex4.y, vertex4.z);
    stroke(color(67, 128, 23));
    line(vertex5.x, vertex5.y, vertex5.z, vertex6.x, vertex6.y, vertex6.z);
  }
  
  private void updateHeadInclination(){
    PVector vectorFromNeckToHead = PVector.sub(this.joints[HEAD].estimatedPosition, this.joints[NECK].estimatedPosition);
    if(this.scene.floor.isCalibrated){
      this.headInclination = asin(PVector.dot(vectorFromNeckToHead, this.scene.floor.basisVectorX)/vectorFromNeckToHead.mag());
    } else {
      this.headInclination = asin(vectorFromNeckToHead.x/vectorFromNeckToHead.mag());
    }
    //println("headInclination: "+this.headInclination);
  }
  
  private void updateShoulderTension(){
    PVector vectorFromSpineShoulderToLeftShoulder = PVector.sub(this.joints[SHOULDER_LEFT].estimatedPosition, this.joints[SPINE_SHOULDER].estimatedPosition);
    PVector vectorFromSpineShoulderToRightShoulder = PVector.sub(this.joints[SHOULDER_RIGHT].estimatedPosition, this.joints[SPINE_SHOULDER].estimatedPosition);
    PVector vectorFromSpineShoulderToNeck = PVector.sub(this.joints[NECK].estimatedPosition, this.joints[SPINE_SHOULDER].estimatedPosition);
    PVector resultantVectorFromSpineShoulderToShoulders = PVector.add(vectorFromSpineShoulderToLeftShoulder, vectorFromSpineShoulderToRightShoulder);
    this.shoulderTension = PVector.dot(resultantVectorFromSpineShoulderToShoulders, vectorFromSpineShoulderToNeck);
  }
  
  private void drawCenterOfMass(float size){
    pushMatrix();
    translate(reScaleX(this.centerOfMass.x, "drawCenterOfMass"), 
              reScaleY(this.centerOfMass.y, "drawCenterOfMass"), 
              reScaleZ(this.centerOfMass.z, "drawCenterOfMass"));
    noStroke();
    fill(125, 0, 200, 128);
    sphere(5);
    noFill();
    stroke(75, 0, 150, 128);
    strokeWeight(1);
    sphereDetail(14);
    sphere(size*this.dispersion);
    popMatrix();
  }
    
/**
 * Draws a shallow sphere around each hand with its respective size.
 */
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
        translate(reScaleX(this.joints[HAND_LEFT].estimatedPosition.x, "skeleton.drawHandRadius"),
                  reScaleY(this.joints[HAND_LEFT].estimatedPosition.y, "skeleton.drawHandRadius"),
                  reScaleZ(this.joints[HAND_LEFT].estimatedPosition.z, "skeleton.drawHandRadius"));
      } else{ // right hand
        translate(reScaleX(this.joints[HAND_RIGHT].estimatedPosition.x, "skeleton.drawHandRadius"),
                  reScaleY(this.joints[HAND_RIGHT].estimatedPosition.y, "skeleton.drawHandRadius"),
                  reScaleZ(this.joints[HAND_RIGHT].estimatedPosition.z, "skeleton.drawHandRadius"));
      }
      sphere(sphereRadius);
      popMatrix();
    }
  }
  
/**
 * Draw the raw hand states received from kinect. Draw nothing if hand is not tracked or unknown.
 */
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
        translate(reScaleX(this.joints[HAND_LEFT].estimatedPosition.x, "skeleton.drawHandStates"),
                  reScaleY(this.joints[HAND_LEFT].estimatedPosition.y, "skeleton.drawHandStates"),
                  reScaleZ(this.joints[HAND_LEFT].estimatedPosition.z, "skeleton.drawHandStates"));
      } else{ // right hand
        translate(reScaleX(this.joints[HAND_RIGHT].estimatedPosition.x, "skeleton.drawHandStates"),
                  reScaleY(this.joints[HAND_RIGHT].estimatedPosition.y, "skeleton.drawHandStates"),
                  reScaleZ(this.joints[HAND_RIGHT].estimatedPosition.z, "skeleton.drawHandStates"));
      }
      sphere(sphereRadius);
      popMatrix();
    }
  }
}

final int SPINE_BASE = 0;
final int SPINE_MID = 1;
final int NECK = 2;
final int HEAD = 3;
final int SHOULDER_LEFT = 4;
final int ELBOW_LEFT = 5;
final int WRIST_LEFT = 6;
final int HAND_LEFT = 7;
final int SHOULDER_RIGHT = 8;
final int ELBOW_RIGHT = 9;
final int WRIST_RIGHT = 10;
final int HAND_RIGHT = 11;
final int HIP_LEFT = 12;
final int KNEE_LEFT = 13;
final int ANKLE_LEFT = 14;
final int FOOT_LEFT = 15;
final int HIP_RIGHT = 16;
final int KNEE_RIGHT = 17;
final int ANKLE_RIGHT = 18;
final int FOOT_RIGHT = 19;
final int SPINE_SHOULDER = 20;
final int HAND_TIP_LEFT = 21;
final int THUMB_LEFT = 22;
final int HAND_TIP_RIGHT = 23;
final int THUMB_RIGHT = 24;
