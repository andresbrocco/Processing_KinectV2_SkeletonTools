/**
 * A skeleton contains all the information related to a specific body. It also contains the algorithms to smooth its movement and draw itself on screen.
 */
public class Skeleton{
  private Scene scene;
  private int indexColor; // Which body is it (color = - M*255*256^2 - C*255*256 - Y*255 -1)
  private int[] skeletonColorRGB = new int[3]; // in RGB
  private color colorEstimated;
  private color colorMeasured;
  private int appearedLastInFrame = 0; // counter to keep track if skeleton is dead or not.
  private float responseTradeoff = 0.2; // Tradeoff between speed and smoothness: close to 0 gives faster responses but more noise. Larger values give lazy skeleton. 
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
    this.appearedLastInFrame = frameCount;
    this.features = new Features(this);
  }
  
/**
 * Receives new raw skeleton data from kinect, smooth its movement and updates its features. 
 * @param kSkeleton raw skeleton data from kinect.
 */
  public void update(KSkeleton kSkeleton){   //<>//
    this.isTracked = kSkeleton.isTracked(); 
    this.measuredHandStates[0] = kSkeleton.getLeftHandState();
    this.measuredHandStates[1] = kSkeleton.getRightHandState();
    this.measureHandRadius();
    KJoint[] kJoints = kSkeleton.getJoints();
    for (int j=0; j<25; j++){
      joints[j].receiveNewMeasurements(kJoints[j]);
    }
    this.smoothSkeleton();
    this.appearedLastInFrame = frameCount;
    this.features.update();
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
  private void smoothSkeleton(){ //<>//
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
  public void draw(boolean drawMeasured, boolean drawJointOrientation, boolean drawBoneRelativeOrientation,  boolean drawHandRadius, boolean drawHandStates){
    for(Bone bone:this.bones){
      bone.draw(drawMeasured, drawBoneRelativeOrientation);
    }
    for(Joint joint:this.joints){
      joint.draw(drawMeasured, drawJointOrientation);
    }
    if(drawHandRadius){
      this.drawHandRadius();
    }
    if(drawHandStates){
      this.drawHandStates();
    }
    // Both below shall be deleted
    // testing relative position to the floor coordinate system:
    //if(this.scene.floor.isCalibrated) this.testingRelativePosition();
    // Testing Steering Wheel:
    this.drawSteeringWheel();
  }
  
/**
 * For testing only, shall be deprecated
 */
  private void drawSteeringWheel(){
    PVector vertex1 = PVector.mult(new PVector(cos(this.features.steeringWheel.yawAngle), 0, sin(this.features.steeringWheel.yawAngle)), 100*this.features.steeringWheel.yawSize);
    PVector vertex2 = PVector.mult(new PVector(-cos(this.features.steeringWheel.yawAngle), 0, -sin(this.features.steeringWheel.yawAngle)), 100*this.features.steeringWheel.yawSize);
    PVector vertex3 = PVector.mult(new PVector(0, cos(this.features.steeringWheel.pitchAngle), sin(this.features.steeringWheel.pitchAngle)), 100*this.features.steeringWheel.pitchSize);
    PVector vertex4 = PVector.mult(new PVector(0, -cos(this.features.steeringWheel.pitchAngle), -sin(this.features.steeringWheel.pitchAngle)), 100*this.features.steeringWheel.pitchSize);
    PVector vertex5 = PVector.mult(new PVector(cos(this.features.steeringWheel.rollAngle), sin(this.features.steeringWheel.rollAngle), 0), 100*this.features.steeringWheel.rollSize);
    PVector vertex6 = PVector.mult(new PVector(-cos(this.features.steeringWheel.rollAngle), -sin(this.features.steeringWheel.rollAngle), 0), 100*this.features.steeringWheel.rollSize);
    strokeWeight(5);
    stroke(color(128, 67, 23));
    line(vertex1.x, vertex1.y, vertex1.z, vertex2.x, vertex2.y, vertex2.z);
    stroke(color(67, 23, 128));
    line(vertex3.x, vertex3.y, vertex3.z, vertex4.x, vertex4.y, vertex4.z);
    stroke(color(67, 128, 23));
    line(vertex5.x, vertex5.y, vertex5.z, vertex6.x, vertex6.y, vertex6.z);
  }
  
/**
 * For testing only, shall be deprecated
 */
  private void testingRelativePosition(){ // For testing
    pushMatrix();
    translate(reScaleX(this.scene.floor.centerPosition.x), reScaleY(this.scene.floor.centerPosition.y), reScaleZ(this.scene.floor.centerPosition.z));
    PVector point = PVector.mult(this.scene.floor.basisVectorX, this.features.leftHandPositionLocal.x).add(PVector.mult(this.scene.floor.basisVectorY, this.features.leftHandPositionLocal.y)).add(PVector.mult(this.scene.floor.basisVectorZ, this.features.leftHandPositionLocal.z));
    line(0, 0, 0, reScaleX(point.x), reScaleY(point.y), reScaleZ(point.z));
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
