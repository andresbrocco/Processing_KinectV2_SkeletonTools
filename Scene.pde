Scene scene = new Scene();
import KinectPV2.*;

KinectPV2 kinect = new KinectPV2(this);

public class Scene{
  private float frameRate_ = 10;
  private float cameraTransX = 600;
  private float cameraTransY = 420;
  private float cameraTransZ = -60;
  private float cameraRotX = -3.5;
  private float cameraRotY = -0.6; 
  private color roomColor = color(0, 0, 0, 150);
  private color backgroundColor = color(128);
  private HashMap<Integer, Skeleton> activeSkeletons = new HashMap<Integer, Skeleton>();
  private float currentDeltaT;
  private float previousDeltaT;
  public Floor floor;
  
  public Scene(){
    this.currentDeltaT = 1/this.frameRate_; // static deltaT
    this.previousDeltaT = this.currentDeltaT; // static deltaT
    this.floor = new Floor(this);
  }
  
  public void init(){
    kinect.enableSkeleton3DMap(true);
    kinect.init();  
  }
  
  public void update(){
    ArrayList<KSkeleton> kSkeletonArray =  kinect.getSkeleton3d();
    for (int bodyNumber = 0; bodyNumber < kSkeletonArray.size(); bodyNumber++){
      KSkeleton kSkeleton = kSkeletonArray.get(bodyNumber);
      if (!activeSkeletons.containsKey(kSkeleton.getIndexColor())){
        this.activeSkeletons.put(kSkeleton.getIndexColor(), new Skeleton(kSkeleton, this));
      }
      Skeleton skeleton = activeSkeletons.get(kSkeleton.getIndexColor());
      skeleton.update(kSkeleton, this.currentDeltaT, this.previousDeltaT);
    }
    this.cleanDeadSkeletons();
  }
  
  private void cleanDeadSkeletons(){
    int timeTolerance = 5; // skeleton abscent for 5 seconds is deleted
    int s = 0;
    int[] skeletonsToRemove = new int[6];
    for(Skeleton skeleton:activeSkeletons.values()){
      if(frameCount - skeleton.appearedLastInFrame > frameRate*timeTolerance){ 
        skeletonsToRemove[s] = skeleton.indexColor;
        s++;
      }
    }
    for(int s_:skeletonsToRemove){
      this.activeSkeletons.remove(s_);  
    }
  }
  
  public void drawOnScreen(boolean drawMeasuredSkeletons, boolean drawJointOrientation, boolean drawBoneRelativeOrientation){  
    background(this.backgroundColor);
    this.setCamera();
    if(!this.activeSkeletons.isEmpty()){
      for (Skeleton skeleton:this.activeSkeletons.values()) {
        this.drawSkeleton(skeleton, drawMeasuredSkeletons, drawJointOrientation, drawBoneRelativeOrientation);
      }
    }
    this.drawKinectFieldOfView();
    if(this.floor.isCalibrated){
      //this.floor.drawPlane();
      this.floor.drawBox();
    } else if(this.floor.isCalibrating && this.floor.enableBoxDraw){
      this.floor.drawBox();
      this.floor.drawData();
    }
  }
  
  private void setCamera(){
    perspective();
    beginCamera();
    camera();
    translate(this.cameraTransX, this.cameraTransY, this.cameraTransZ);
    rotateX(this.cameraRotX);
    rotateY(this.cameraRotY);
    endCamera();
  }
  
  private void drawKinectCoordinateSystem(){
    float size = 0.5; // meters
    strokeWeight(5);
    stroke(255, 0, 0);
    line(0, 0, 0, reScaleX(size), 0, 0); // The Processing's coordinate system is inconsistent (X cross Y != Z)
    stroke(0, 255, 0);
    line(0, 0, 0, 0, reScaleY(size), 0);
    stroke(0, 0, 255);
    line(0, 0, 0, 0, 0, reScaleZ(size));
  }
  
  private void drawKinectFieldOfView(){ 
    this.drawKinectCoordinateSystem();
    // KinectV2
      //Depth camera:
        //vertical FoV: 60 deg. horizontal FoV: 71 deg.
        float verticalFoV = 60;
        float horizontalFoV = 71;
        float minimumDepth = 0.5; // meters
        float maximumDepth = 4; // meters
        float minimumDepthInPixels = reScaleZ(minimumDepth);
        float maximumDepthInPixels = reScaleZ(maximumDepth);
      //Color camera:
        //vertical FoV: 54 deg. horizontal FoV: 84 deg.
    // KinectV1
      //Depth camera:
        //vertical FoV: 46 deg. horizontal FoV: 59 deg.
      //Color camera:
        //vertical FoV: 49 deg. horizontal FoV: 62 deg.
    pushMatrix();
    stroke(this.roomColor);
    strokeWeight(2);
    noFill();
    // Closest face
    beginShape();
    vertex(minimumDepthInPixels*tan(radians(horizontalFoV/2)), minimumDepthInPixels*tan(radians(verticalFoV/2)), minimumDepthInPixels);
    vertex(-minimumDepthInPixels*tan(radians(horizontalFoV/2)), minimumDepthInPixels*tan(radians(verticalFoV/2)), minimumDepthInPixels);
    vertex(-minimumDepthInPixels*tan(radians(horizontalFoV/2)), -minimumDepthInPixels*tan(radians(verticalFoV/2)), minimumDepthInPixels);
    vertex(minimumDepthInPixels*tan(radians(horizontalFoV/2)), -minimumDepthInPixels*tan(radians(verticalFoV/2)), minimumDepthInPixels);
    endShape(CLOSE);
    
    // Farthest face
    beginShape();
    vertex(maximumDepthInPixels*tan(radians(horizontalFoV/2)), maximumDepthInPixels*tan(radians(verticalFoV/2)), maximumDepthInPixels);
    vertex(-maximumDepthInPixels*tan(radians(horizontalFoV/2)), maximumDepthInPixels*tan(radians(verticalFoV/2)), maximumDepthInPixels);
    vertex(-maximumDepthInPixels*tan(radians(horizontalFoV/2)), -maximumDepthInPixels*tan(radians(verticalFoV/2)), maximumDepthInPixels);
    vertex(maximumDepthInPixels*tan(radians(horizontalFoV/2)), -maximumDepthInPixels*tan(radians(verticalFoV/2)), maximumDepthInPixels);
    endShape(CLOSE);
    
    // Lines Connecting faces
    beginShape(LINES);
    vertex(minimumDepthInPixels*tan(radians(horizontalFoV/2)), minimumDepthInPixels*tan(radians(verticalFoV/2)), minimumDepthInPixels);
    vertex(maximumDepthInPixels*tan(radians(horizontalFoV/2)), maximumDepthInPixels*tan(radians(verticalFoV/2)), maximumDepthInPixels);
    vertex(-minimumDepthInPixels*tan(radians(horizontalFoV/2)), minimumDepthInPixels*tan(radians(verticalFoV/2)), minimumDepthInPixels);
    vertex(-maximumDepthInPixels*tan(radians(horizontalFoV/2)), maximumDepthInPixels*tan(radians(verticalFoV/2)), maximumDepthInPixels);
    vertex(-minimumDepthInPixels*tan(radians(horizontalFoV/2)), -minimumDepthInPixels*tan(radians(verticalFoV/2)), minimumDepthInPixels);
    vertex(-maximumDepthInPixels*tan(radians(horizontalFoV/2)), -maximumDepthInPixels*tan(radians(verticalFoV/2)), maximumDepthInPixels);
    vertex(minimumDepthInPixels*tan(radians(horizontalFoV/2)), -minimumDepthInPixels*tan(radians(verticalFoV/2)), minimumDepthInPixels);
    vertex(maximumDepthInPixels*tan(radians(horizontalFoV/2)), -maximumDepthInPixels*tan(radians(verticalFoV/2)), maximumDepthInPixels);
    endShape();
    popMatrix();
  }
  
  void drawSkeleton(Skeleton skeleton, boolean drawMeasuredSkeleton, boolean drawJointOrientation, boolean drawBoneRelativeOrientation){
    color colorEstimated = color(skeleton.skeletonColorRGB[0], skeleton.skeletonColorRGB[1], skeleton.skeletonColorRGB[2], 170);
    color colorMeasured = color(skeleton.skeletonColorRGB[0], skeleton.skeletonColorRGB[1], skeleton.skeletonColorRGB[2], 85);
    for(Bone bone:skeleton.bones){
      bone.drawBone(colorEstimated, colorMeasured, drawMeasuredSkeleton);
      if(drawBoneRelativeOrientation){
        bone.drawRelativeOrientation(10);
      }
    }
    for(Joint joint:skeleton.joints){
      joint.drawPosition(colorEstimated);
      if(drawJointOrientation && !joint.isEndJoint){
        joint.drawOrientation(true, true, 15); // drawEstimated, drawMeasured
      }
    }
    //skeleton.drawHandStates();
    //skeleton.drawHandRadius();
  }
}

void mouseDragged() {
  if(mouseButton == CENTER){
    scene.cameraRotX = scene.cameraRotX - (mouseY - pmouseY)*PI/height;
    scene.cameraRotY = scene.cameraRotY - (mouseX - pmouseX)*PI/width;
  }
  if(mouseButton == LEFT){
    scene.cameraTransX = scene.cameraTransX + (mouseX - pmouseX);
    scene.cameraTransY = scene.cameraTransY + (mouseY - pmouseY);
  }
  println("RotX: "+scene.cameraRotX);
  println("RotY: "+scene.cameraRotY);
  println("cameraTransX: "+scene.cameraTransX);
  println("cameraTransY: "+scene.cameraTransY);
}

void mouseWheel(MouseEvent event) {
  float zoom = event.getCount();
  if(zoom < 0){
    scene.cameraTransZ = scene.cameraTransZ + 30;
  }else{
    scene.cameraTransZ = scene.cameraTransZ - 30;
  }
  println("cameraTransZ: "+scene.cameraTransZ);
}
