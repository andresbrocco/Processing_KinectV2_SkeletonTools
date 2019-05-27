/*
  Fazer cabeçalho em todos
*/

void setup()
{
  size(600, 600, P3D);
  frameRate(scene.frameRate_);
  scene.init();
}

void draw()
{
  scene.update();
  scene.drawOnScreen(false, false, true); // drawMeasuredSkeletons, drawJointOrientation, drawBoneRelativeOrientation
  communication.sendScene(scene);
}
