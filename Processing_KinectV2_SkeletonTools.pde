/*
  Fazer cabeçalho em todos
*/
boolean drawSkeletonTool = true;
Scene scene = new Scene();

int pdPort = 3000;
int myPort = 3001;
Communication communication = new Communication("127.0.0.1", pdPort, myPort);

void setup()
{
  frameRate(scene.frameRate_);
  size(600, 600, P3D);
  scene.init();
}

void draw()
{
  for(Skeleton skeleton:scene.activeSkeletons.values()){ //example of consulting feature
    println("distanceBetweenHands: "+ skeleton.features.distanceBetweenHands);
  }
  scene.update();
  if(drawSkeletonTool){
    scene.drawOnScreen(false, false, true); // drawMeasuredSkeletons, drawJointOrientation, drawBoneRelativeOrientation  
  } else{
    // Your animation algorithm should be placed here
    background(color(128));
  }
  communication.sendScene(scene);
}

void keyPressed(){
  // Press f to enter floor calibration process
  if(key == 'f'){
    if(scene.floor.isCalibrating){
      scene.floor.isCalibrating = false;
      scene.floor.isCalibrated = true;
      println("Floor calibration complete!");
    }else{
      thread("calibrateFloor");
    }
  }
  // Press d to draw Skeleton Tool
  if(key == 'd'){
    if(drawSkeletonTool){
      drawSkeletonTool = false;
      println("drawSkeletonTool disabled");
    }else{
      drawSkeletonTool = true;
      println("drawSkeletonTool enabled");
    }
  }
  if(scene.floor.isWaitingForUser && (key==ENTER || key==RETURN)){
    scene.floor.isCalibrating  = true;
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
}

void mouseWheel(MouseEvent event) {
  float zoom = event.getCount();
  if(zoom < 0){
    scene.cameraTransZ = scene.cameraTransZ + 30;
  }else{
    scene.cameraTransZ = scene.cameraTransZ - 30;
  }
}
