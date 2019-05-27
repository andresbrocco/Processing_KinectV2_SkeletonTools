import Jama.*; // Java Matrix Library: https://math.nist.gov/javanumerics/jama/

class Floor{
  private Scene scene;
  private int maximumFeetPositions = 500;
  private Matrix historyOfFeetPositions = new Matrix(maximumFeetPositions, 3);
  private PVector averageFeetPosition = new PVector();
  private SingularValueDecomposition svd;
  private PVector singularValues = new PVector();
  private PVector basisVector1 = new PVector();
  private PVector basisVector2 = new PVector();
  private PVector basisVector3 = new PVector();
  private Quaternion orientation;
  private float[][] rotationMatrix = new float[3][3];
  private int indexToBeUpdated = 0;
  private boolean bufferIsFull = false;
  private float boxDimension = 2; // "meters"
  private boolean isWaitingForUser = false;
  private boolean isCalibrating = false;
  private boolean isCalibrated = false;
  private boolean enableBoxDraw = false;
  private Plane plane;
  
  public Floor(Scene scene){
    this.scene = scene;
  }
  
  public void controlledCalibration(){
    println("Floor Calibration instructions: ");
    println("The calibrator needs at least 10 snapshots of the skeleton in different spots of the room to have a good estimate.");
    println("The reccomended position is upright with legs opened at 45~60 deg.");
    println("The opened legs partially overcomes the issue with reflecting floors.");
    println("Get in position and clap to get a snapshot.");
    this.isCalibrating = true;
    int snapshotCount = 0;
    while(this.isCalibrating){
      for(Skeleton skeleton:this.scene.activeSkeletons.values()){
        if(skeleton.features.distanceBetweenHands < 0.1){
          snapshotCount++;
          println("Snapshot count: " + snapshotCount);
          this.addSkeletonFeet(skeleton);
          this.calculateFloor();
        }
      }
      delay(100); // minimum time between snapshots
    }
  }

  public void timedCalibration(){
    println("Floor Calibration instructions: ");
    println("The calibrator needs at least 10 snapshots of the skeleton in different spots of the room to have a good estimate.");
    println("The reccomended position is upright with legs opened at 45~60 deg.");
    println("The opened legs partially overcomes the issue with reflecting floors.");
    println("There will be 5 seconds between each snapshot to walk to a new spot.");
    println("Press ENTER when you are ready");
    this.isWaitingForUser = true;
    int startTime = millis();
    while(true){ // 10 seconds to start the calibration
      if(millis()-startTime < 10000){
        if(this.isCalibrating){
          break;
        }
      } else {
      println("Sorry, timed out! Start calibration again if you want...");
      break;
      }
      delay(100);
    }
    int snapshotCount = 0;
    snapshotLoop:
    while(this.isCalibrating){
      for(int countdown = 2; countdown>0; countdown--){
        println("Snapshot in "+ countdown + " seconds");
        delay(1000);      
        if(!this.isCalibrating){
          this.isCalibrated = true;
          break snapshotLoop;
        }
      }
      this.addScene();
      snapshotCount++;
      println("Snapshot count: " + snapshotCount);
    }
  }
  
  public void addScene(){
    for(Skeleton skeleton:this.scene.activeSkeletons.values()){
      this.addSkeletonFeet(skeleton);
    }
    this.calculateFloor();
  }
  
  private void addSkeletonFeet(Skeleton skeleton){
    if(!bufferIsFull){
      if(skeleton.joints[15].trackingState == 2){ // if FootLeft is tracked 
        this.addFoot(skeleton.joints[15]);  
      }
      if(skeleton.joints[19].trackingState == 2){ // if FootRight is tracked 
        this.addFoot(skeleton.joints[19]);  
      }
    }
  }
  
  private void addFoot(Joint footJoint){ 
    if(this.indexToBeUpdated < 3){
      this.historyOfFeetPositions.set(this.indexToBeUpdated, 0, footJoint.estimatedPosition.x);
      this.historyOfFeetPositions.set(this.indexToBeUpdated, 1, footJoint.estimatedPosition.y);
      this.historyOfFeetPositions.set(this.indexToBeUpdated, 2, footJoint.estimatedPosition.z);
      println("added index: "+ this.indexToBeUpdated);
      this.indexToBeUpdated++;
    } 
    else {
      if(this.indexToBeUpdated < maximumFeetPositions){ 
        this.historyOfFeetPositions.set(this.indexToBeUpdated, 0, footJoint.estimatedPosition.x);
        this.historyOfFeetPositions.set(this.indexToBeUpdated, 1, footJoint.estimatedPosition.y);
        this.historyOfFeetPositions.set(this.indexToBeUpdated, 2, footJoint.estimatedPosition.z);
        
        this.updateAverageFeetPosition();
        println("added index: "+ this.indexToBeUpdated);
        this.indexToBeUpdated++;
      }
      else {
        this.bufferIsFull = true;
        println("buffer is full");
      }
    }
  }
  
  private void updateAverageFeetPosition(){
    double[] sumOfFeetPositions = new double[3];
    double[] averageFeetPosition = new double[3];
    float[] feetPositionVariance = new float[3];
    float[] feetPositionStandardDeviation = new float[3];
    for(int col=0; col<3; col++){
      for (int row=0; row<this.indexToBeUpdated; row++){
        sumOfFeetPositions[col] = sumOfFeetPositions[col]+this.historyOfFeetPositions.get(row, col);
      }
      averageFeetPosition[col] = sumOfFeetPositions[col]/this.indexToBeUpdated; 
      for (int row=0; row<this.indexToBeUpdated; row++){
        feetPositionVariance[col] = feetPositionVariance[col]+sq((float)(this.historyOfFeetPositions.get(row, col)-averageFeetPosition[col]));
      }
      feetPositionStandardDeviation[col] = sqrt(feetPositionVariance[col]/(this.indexToBeUpdated-1)); 
    }
    this.averageFeetPosition = new PVector((float)averageFeetPosition[0], (float)averageFeetPosition[1], (float)averageFeetPosition[2]);
  }
  
  private Matrix basisVectorsToFloorCoordinateSystem(PVector basisVector1, PVector basisVector2, PVector basisVector3){
    PVector basisVectorX;
    PVector basisVectorY;
    if(abs(basisVector1.x) >= abs(basisVector2.x) && abs(basisVector1.x) >= abs(basisVector3.x)){
      println("casex: 1");
      basisVectorX = PVector.mult(basisVector1, Math.signum(basisVector1.x)); 
    } else if(abs(basisVector2.x) >= abs(basisVector3.x)){
      println("casex: 2");
      basisVectorX = PVector.mult(basisVector2, Math.signum(basisVector2.x));
    } else{
      println("casex: 3");
      basisVectorX = PVector.mult(basisVector3, Math.signum(basisVector3.x));
    }
    if(abs(basisVector1.y) >= abs(basisVector2.y) && abs(basisVector1.y) >= abs(basisVector3.y)){
      basisVectorY = PVector.mult(basisVector1, Math.signum(basisVector1.y));
      println("casey: 1");
    } else if(abs(basisVector2.y) >= abs(basisVector3.y)){
      println("casey: 2");
      basisVectorY = PVector.mult(basisVector2, Math.signum(basisVector2.y));
    } else{
      println("casey: 3");
      basisVectorY = PVector.mult(basisVector3, Math.signum(basisVector3.y)); 
    }
    PVector basisVectorZ = basisVectorX.cross(basisVectorY);
    
    println("basisVectors checker: "+basisVectorX.cross(basisVectorY).dot(basisVectorZ));
    Matrix coordinateSystem = new Matrix(3, 3);
    coordinateSystem.set(0, 0, (double)basisVectorX.x);
    coordinateSystem.set(1, 0, (double)basisVectorX.y);
    coordinateSystem.set(2, 0, (double)basisVectorX.z);
    coordinateSystem.set(0, 1, (double)basisVectorY.x);
    coordinateSystem.set(1, 1, (double)basisVectorY.y);
    coordinateSystem.set(2, 1, (double)basisVectorY.z);
    coordinateSystem.set(0, 2, (double)basisVectorZ.x);
    coordinateSystem.set(1, 2, (double)basisVectorZ.y);
    coordinateSystem.set(2, 2, (double)basisVectorZ.z);
    return coordinateSystem;
  }

  private void calculateFloor(){
    if(this.indexToBeUpdated > 3){
      Matrix filledHistoryOfFeetPositions = this.historyOfFeetPositions.getMatrix(0, this.indexToBeUpdated-1, 0, 2);
      this.svd = filledHistoryOfFeetPositions.svd();
      double[] singularValues = this.svd.getSingularValues();
      this.singularValues = new PVector((float)singularValues[0], (float)singularValues[1], (float)singularValues[2]);
      println("singularValues: "+this.singularValues);
      this.singularValues.normalize();
      
      double[][] basisVectors = this.svd.getV().getArray();
      println("this.svd.getV().det(): "+this.svd.getV().det()); // the resulting vectors are unitary!
      this.basisVector1 = new PVector((float)basisVectors[0][0], (float)basisVectors[1][0], (float)basisVectors[2][0]);
      this.basisVector2 = new PVector((float)basisVectors[0][1], (float)basisVectors[1][1], (float)basisVectors[2][1]);
      this.basisVector3 = new PVector((float)basisVectors[0][2], (float)basisVectors[1][2], (float)basisVectors[2][2]);
      Matrix floorCoordinateSystem = basisVectorsToFloorCoordinateSystem(this.basisVector1, this.basisVector2, this.basisVector3);
      this.plane = new Plane(this.basisVector1, this.basisVector2, this.averageFeetPosition);
      this.orientation = rotationMatrixToQuaternion(floorCoordinateSystem);
      this.enableBoxDraw = true;
    }
  }
  
  public void drawPlane(){
    PVector floorCorner1 = PVector.add(this.averageFeetPosition, PVector.mult(this.basisVector1, this.boxDimension)).add(PVector.mult(this.basisVector2, this.boxDimension));
    PVector floorCorner2 = PVector.add(this.averageFeetPosition, PVector.mult(this.basisVector1, -this.boxDimension)).add(PVector.mult(this.basisVector2, this.boxDimension));
    PVector floorCorner3 = PVector.add(this.averageFeetPosition, PVector.mult(this.basisVector1, -this.boxDimension)).add(PVector.mult(this.basisVector2, -this.boxDimension));
    PVector floorCorner4 = PVector.add(this.averageFeetPosition, PVector.mult(this.basisVector1, this.boxDimension)).add(PVector.mult(this.basisVector2, -this.boxDimension));

    fill(90, 30, 90, 50);
    beginShape();
    vertex(reScaleX(floorCorner1.x), reScaleY(floorCorner1.y), reScaleZ(floorCorner1.z));
    vertex(reScaleX(floorCorner2.x), reScaleY(floorCorner2.y), reScaleZ(floorCorner2.z));
    vertex(reScaleX(floorCorner3.x), reScaleY(floorCorner3.y), reScaleZ(floorCorner3.z));
    vertex(reScaleX(floorCorner4.x), reScaleY(floorCorner4.y), reScaleZ(floorCorner4.z));
    endShape(CLOSE);
  }
  
  public void drawBox(){
    if(this.indexToBeUpdated > 3){
      float floorWidth = this.boxDimension*this.singularValues.x;
      float floorHeight = this.boxDimension*this.singularValues.y;
      float floorThickness = this.boxDimension*this.singularValues.z;
      
      // Lower face:
      PVector floorCorner1 = PVector.add(this.averageFeetPosition, PVector.mult(this.basisVector1, floorHeight)).add(PVector.mult(this.basisVector2, floorWidth)).add(PVector.mult(this.basisVector3, floorThickness));
      PVector floorCorner2 = PVector.add(this.averageFeetPosition, PVector.mult(this.basisVector1, -floorHeight)).add(PVector.mult(this.basisVector2, floorWidth)).add(PVector.mult(this.basisVector3, floorThickness));
      PVector floorCorner3 = PVector.add(this.averageFeetPosition, PVector.mult(this.basisVector1, -floorHeight)).add(PVector.mult(this.basisVector2, -floorWidth)).add(PVector.mult(this.basisVector3, floorThickness));
      PVector floorCorner4 = PVector.add(this.averageFeetPosition, PVector.mult(this.basisVector1, floorHeight)).add(PVector.mult(this.basisVector2, -floorWidth)).add(PVector.mult(this.basisVector3, floorThickness));
  
      // Upper Face:
      PVector floorCorner5 = PVector.add(this.averageFeetPosition, PVector.mult(this.basisVector1, floorHeight)).add(PVector.mult(this.basisVector2, floorWidth)).add(PVector.mult(this.basisVector3, -floorThickness));
      PVector floorCorner6 = PVector.add(this.averageFeetPosition, PVector.mult(this.basisVector1, -floorHeight)).add(PVector.mult(this.basisVector2, floorWidth)).add(PVector.mult(this.basisVector3, -floorThickness));
      PVector floorCorner7 = PVector.add(this.averageFeetPosition, PVector.mult(this.basisVector1, -floorHeight)).add(PVector.mult(this.basisVector2, -floorWidth)).add(PVector.mult(this.basisVector3, -floorThickness));
      PVector floorCorner8 = PVector.add(this.averageFeetPosition, PVector.mult(this.basisVector1, floorHeight)).add(PVector.mult(this.basisVector2, -floorWidth)).add(PVector.mult(this.basisVector3, -floorThickness));
      
      noFill();
      
      // Lower Face:
      beginShape();
      vertex(reScaleX(floorCorner1.x), reScaleY(floorCorner1.y), reScaleZ(floorCorner1.z));
      vertex(reScaleX(floorCorner2.x), reScaleY(floorCorner2.y), reScaleZ(floorCorner2.z));
      vertex(reScaleX(floorCorner3.x), reScaleY(floorCorner3.y), reScaleZ(floorCorner3.z));
      vertex(reScaleX(floorCorner4.x), reScaleY(floorCorner4.y), reScaleZ(floorCorner4.z));
      endShape(CLOSE);
      
      // Upper Face:
      beginShape();
      vertex(reScaleX(floorCorner5.x), reScaleY(floorCorner5.y), reScaleZ(floorCorner5.z));
      vertex(reScaleX(floorCorner6.x), reScaleY(floorCorner6.y), reScaleZ(floorCorner6.z));
      vertex(reScaleX(floorCorner7.x), reScaleY(floorCorner7.y), reScaleZ(floorCorner7.z));
      vertex(reScaleX(floorCorner8.x), reScaleY(floorCorner8.y), reScaleZ(floorCorner8.z));
      endShape(CLOSE);
      
      // Connecting Lines:
      beginShape(LINES);
      vertex(reScaleX(floorCorner1.x), reScaleY(floorCorner1.y), reScaleZ(floorCorner1.z));
      vertex(reScaleX(floorCorner5.x), reScaleY(floorCorner5.y), reScaleZ(floorCorner5.z));
      vertex(reScaleX(floorCorner2.x), reScaleY(floorCorner2.y), reScaleZ(floorCorner2.z));
      vertex(reScaleX(floorCorner6.x), reScaleY(floorCorner6.y), reScaleZ(floorCorner6.z));
      vertex(reScaleX(floorCorner3.x), reScaleY(floorCorner3.y), reScaleZ(floorCorner3.z));
      vertex(reScaleX(floorCorner7.x), reScaleY(floorCorner7.y), reScaleZ(floorCorner7.z));
      vertex(reScaleX(floorCorner4.x), reScaleY(floorCorner4.y), reScaleZ(floorCorner4.z));
      vertex(reScaleX(floorCorner8.x), reScaleY(floorCorner8.y), reScaleZ(floorCorner8.z));
      endShape();
      this.drawCoordinateSystem(true, false);
    }
  }
  
  private void drawCoordinateSystem(boolean fromQuaternion, boolean fromSVDBasis){
    if(fromQuaternion){
      PVector coordinateSystemDirectionX = qMult(qMult(this.orientation, new Quaternion(0, 1, 0, 0)), qConjugate(this.orientation)).vector; 
      PVector coordinateSystemDirectionY = qMult(qMult(this.orientation, new Quaternion(0, 0, 1, 0)), qConjugate(this.orientation)).vector; 
      PVector coordinateSystemDirectionZ = qMult(qMult(this.orientation, new Quaternion(0, 0, 0, 1)), qConjugate(this.orientation)).vector; 
      //println("coordinateSystemDirectionX mag: "+coordinateSystemDirectionX.mag());
      pushMatrix();
      translate(reScaleX(this.averageFeetPosition.x), reScaleY(this.averageFeetPosition.y), reScaleZ(this.averageFeetPosition.z));
      strokeWeight(5);
      float size = 0.5; // meters
      stroke(255, 0, 0, 170);
      line(0, 0, 0, reScaleX(size*coordinateSystemDirectionX.x), reScaleY(size*coordinateSystemDirectionX.y), reScaleZ(size*coordinateSystemDirectionX.z)); // The Processing's coordinate system is inconsistent (X cross Y != Z)
      stroke(0, 255, 0, 170);
      line(0, 0, 0, reScaleX(size*coordinateSystemDirectionY.x), reScaleY(size*coordinateSystemDirectionY.y), reScaleZ(size*coordinateSystemDirectionY.z));
      stroke(0, 0, 255, 170);
      line(0, 0, 0, reScaleX(size*coordinateSystemDirectionZ.x), reScaleY(size*coordinateSystemDirectionZ.y), reScaleZ(size*coordinateSystemDirectionZ.z));
      popMatrix();
    }
    if(fromSVDBasis){
      pushMatrix();
      translate(reScaleX(this.averageFeetPosition.x), reScaleY(this.averageFeetPosition.y), reScaleZ(this.averageFeetPosition.z));
      strokeWeight(5);
      float size = 0.5; // meters
      stroke(255, 0, 0, 170);
      line(0, 0, 0, reScaleX(size*this.basisVector1.x), reScaleY(size*basisVector1.y), reScaleZ(size*basisVector1.z)); // The Processing's coordinate system is inconsistent (X cross Y != Z)
      stroke(0, 255, 0, 170);
      line(0, 0, 0, reScaleX(size*basisVector2.x), reScaleY(size*basisVector2.y), reScaleZ(size*basisVector2.z));
      stroke(0, 0, 255, 170);
      line(0, 0, 0, reScaleX(size*basisVector3.x), reScaleY(size*basisVector3.y), reScaleZ(size*basisVector3.z));
      popMatrix();
    }
  }
  
  public void drawData(){
    sphereDetail(6);
    for(int j=0; j<this.indexToBeUpdated; j++){
      pushMatrix();
      translate(reScaleX((float)this.historyOfFeetPositions.get(j, 0)), reScaleY((float)this.historyOfFeetPositions.get(j, 1)), reScaleZ((float)this.historyOfFeetPositions.get(j, 2)));
      fill(255, 0, 128, 128);
      noStroke();
      sphere(5);
      popMatrix();
    }
    pushMatrix();
    translate(reScaleX(averageFeetPosition.x), reScaleY(averageFeetPosition.y), reScaleZ(averageFeetPosition.z));
    fill(255, 128, 64, 128);
    noStroke();
    sphere(5);
    popMatrix();    
  }
}

void calibrateFloor(){ // This method exists to make possible to calibrate on another thread other than the draw() loop.
  scene.floor.timedCalibration();
  //scene.floor.controlledCalibration();
}
