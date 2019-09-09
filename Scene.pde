import KinectPV2.*;

KinectPV2 kinect = new KinectPV2(this);
  
/**
 * The scene contains all the environment relative to the kinect, from the floor to the active skeletons.
 */
public class Scene{
  public Floor floor;
  private final float frameRate_ = 10;
  private float cameraTransX = 600;
  private float cameraTransY = 420;
  private float cameraTransZ = -60;
  private float cameraRotX = -3.5;
  private float cameraRotY = -0.6; 
  private float cameraRotZ = 0; 
  private color roomColor = color(0, 0, 0, 150);
  private color backgroundColor = color(128);
  private HashMap<Integer, Skeleton> activeSkeletons = new HashMap<Integer, Skeleton>();
  private float currentDeltaT;
  private float previousDeltaT;
  public int numberOfSkeletons = 0;
  public boolean saveSession = true;
  public String sessionName = "";
  public PrintWriter savingOutput;
  public boolean drawScene = true;
  public boolean drawMeasured = false;
  public boolean drawBoneRelativeOrientation = false;
  public boolean drawJointOrientation = false;
  public boolean drawHandRadius = false;
  public boolean drawHandStates = false;
  public boolean drawPollock = false;
  public boolean drawRondDuBras = false;
  public boolean drawMomentum = false;
  public boolean drawCenterOfMass = false;
  public boolean loadFloorCalibration = true;
  
  public Scene(){
    this.currentDeltaT = 1/this.frameRate_; 
    this.previousDeltaT = this.currentDeltaT;
    this.floor = new Floor(this);
  }
  
  public void init(){
    if(this.loadFloorCalibration) selectInput("Choose a calibrated floor CSV File:", "loadFloorCalibrationThread");
    if(this.saveSession) thread("startSavingSceneThread");
    kinect.enableSkeleton3DMap(true);
    kinect.init();  
  }
  
  public void startSavingScene(){
    println("Enter session name:");
    userTextInput = "";
    gettingUserTextInput = true;
    while(gettingUserTextInput) delay(100);
    this.sessionName = userTextInput;
    if(this.sessionName == ""){
      println("Invalid session name. Session will not be saved.");
      this.saveSession = false;
    } else {
      this.savingOutput = createWriter("savedSessions/"+this.sessionName+"/header.txt");
      this.savingOutput.println("frameRate: "+this.frameRate_);
      if(this.floor.isCalibrated){
        this.savingOutput.println("Floor calibration file used: "+this.floor.selectedCalibrationFilePath);
        this.savingOutput.println("Floor center position: "+this.floor.centerPosition.x+" "+this.floor.centerPosition.y+" "+this.floor.centerPosition.z);
        this.savingOutput.println("Floor orientation: "+this.floor.orientation.real+" "+this.floor.orientation.vector.x+" "+this.floor.orientation.vector.y+" "+this.floor.orientation.vector.z);
        this.savingOutput.println("Floor dimensions: " + this.floor.dimensions.x + " " + this.floor.dimensions.y + " " + this.floor.dimensions.z);
      } else{
        this.savingOutput.println("Floor was not calibrated");
      }
      this.savingOutput.flush();
      this.savingOutput.close();
    }
  }
  
/**
 * Get new data from kinect and call its skeletons to update.
 */
  public void update(){
    if(!this.saveSession || this.sessionName!="") {
      this.previousDeltaT = this.currentDeltaT;
      this.currentDeltaT = 1/frameRate;
      ArrayList<KSkeleton> kSkeletonArray = kinect.getSkeleton3d();
      for (int bodyNumber = 0; bodyNumber < kSkeletonArray.size(); bodyNumber++){
        KSkeleton kSkeleton = kSkeletonArray.get(bodyNumber);
        if (!activeSkeletons.containsKey(kSkeleton.getIndexColor())){ // New skeleton received
          this.numberOfSkeletons++;
          this.activeSkeletons.put(kSkeleton.getIndexColor(), new Skeleton(kSkeleton, this));
        }
        Skeleton skeleton = activeSkeletons.get(kSkeleton.getIndexColor());
        skeleton.update(kSkeleton);
      }
      this.cleanDeadSkeletons();
    }
  }
  
  
/**
 * If a skeleton is abscent for 5 seconds, it is deleted.
 */
  private void cleanDeadSkeletons(){
    int timeTolerance = 5; // seconds
    int s = 0;
    int[] skeletonsToRemove = new int[6];
    for(Skeleton skeleton:activeSkeletons.values()){
      if(frameCount - skeleton.appearedLastInFrame > frameRate*timeTolerance){
        if(this.saveSession) skeleton.savingOutput.close();
        skeletonsToRemove[s] = skeleton.indexColor;
        s++;
      }
    }
    for(int s_:skeletonsToRemove){
      this.activeSkeletons.remove(s_);  
    }
  }
    
/**
 * Set camera, draw skeletons, kinect field of view and floor (if calibrated).
 */
  public void draw(){  
    background(this.backgroundColor);
    this.setCamera();
    if(!this.activeSkeletons.isEmpty()){
      for (Skeleton skeleton:this.activeSkeletons.values()) {
        skeleton.draw(this.drawMeasured, this.drawJointOrientation, this.drawBoneRelativeOrientation, this.drawHandRadius, this.drawHandStates, this.drawPollock, this.drawRondDuBras, this.drawMomentum, this.drawCenterOfMass);
      }
    }
    this.drawKinectFieldOfView();
    this.floor.draw(true, true, true); // coordinateSystem, box, plane
  }
  
/**
 * Set camera position and orientation.
 */
  private void setCamera(){
    perspective();
    beginCamera();
    camera();
    translate(this.cameraTransX, this.cameraTransY, this.cameraTransZ);
    /* Testing Steering Wheel rotating the scene:
    for (Skeleton skeleton:this.activeSkeletons.values()) {
      this.cameraRotX = this.cameraRotX + skeleton.steeringWheel.pitchStep;
      this.cameraRotY = this.cameraRotY + skeleton.steeringWheel.yawStep;
      this.cameraRotZ = this.cameraRotZ + skeleton.steeringWheel.rollStep;
    }*/
    rotateX(this.cameraRotX);
    rotateY(this.cameraRotY);
    //rotateZ(this.cameraRotZ);
    endCamera();
  }
  
/**
 * Draw kinect coordinate system on screen. (actually the X axis is pointing to the wrong side, I don't know why)
 */
  private void drawKinectCoordinateSystem(){
    float size = 0.5; // meters
    strokeWeight(5);
    stroke(255, 0, 0);
    line(0, 0, 0, reScaleX(size, "scene.drawKinectCoordinateSystem"), 0, 0); // The Processing's coordinate system is inconsistent (X cross Y != Z)
    stroke(0, 255, 0);
    line(0, 0, 0, 0, reScaleY(size, "scene.drawKinectCoordinateSystem"), 0);
    stroke(0, 0, 255);
    line(0, 0, 0, 0, 0, reScaleZ(size, "scene.drawKinectCoordinateSystem"));
  }
  
/**
 * Draw kinect Field of View.
 */
  private void drawKinectFieldOfView(){ 
    this.drawKinectCoordinateSystem();
    // KinectV2
      //Depth camera:
        //vertical FoV: 60 deg. horizontal FoV: 71 deg.
        float verticalFoV = 60;
        float horizontalFoV = 71;
        float minimumDepth = 0.5; // meters
        float maximumDepth = 4; // meters
        float minimumDepthInPixels = reScaleZ(minimumDepth, "scene.drawKinectFieldOfView");
        float maximumDepthInPixels = reScaleZ(maximumDepth, "scene.drawKinectFieldOfView");
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
}

/**
 * This method exists to make possible to start saving the scene on another thread other than the setup() loop, because it has a limit of 5000ms to finish before raising an error.
 */
void startSavingSceneThread(){ 
  scene.startSavingScene();
}
