/*
  ToDo: JavaDoc
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
    //println("distanceBetweenHands: "+ skeleton.features.distanceBetweenHands);
  }
  
  scene.update();
  
  if(scene.drawScene){
    scene.draw(); // measuredSkeletons, jointOrientation, boneRelativeOrientation, handRadius, handStates
  } else{
    // Your animation algorithm should be placed here
    background(color(128));
  }
  communication.sendScene(scene);
}

void keyPressed(){
  if(key == 'f') scene.floor.manageCalibration();
  if(key == 's') scene.drawScene = !scene.drawScene;
  if(key == 'm') scene.drawMeasured = !scene.drawMeasured;
  if(key == 'b') scene.drawBoneRelativeOrientation = !scene.drawBoneRelativeOrientation;
  if(key == 'j') scene.drawJointOrientation = !scene.drawJointOrientation;
  if(key == 'h') scene.drawHandRadius = !scene.drawHandRadius;
  if(key == 'H') scene.drawHandStates = !scene.drawHandStates;
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
