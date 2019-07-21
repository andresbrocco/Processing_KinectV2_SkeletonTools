/**
 * RondDuBras: Trigger Affordance that quantifies the "round" movement of each hand.
 * TODO: acertar o metodo para desenhar a face ativada (remover a logica de rotacao com quaternion, e fazer hard-coded)
 */
public class RondDuBras{
  private final float crossProductMagnitudeThreshold = 0.75;
  private int fadeOutTime = 1000; // millisseconds
  String whichHand; // LEFT or RIGHT
  private Joint handJoint;
  private Joint shoulderJoint;
  private Joint spineShoulderJoint;
  private PVector shoulderToHandPosition;
  private PVector shoulderToHandVelocity = new PVector(0, 0, 0);
  private PVector crossProduct = new PVector(0, 0, 0);
  private PVector currentCrossProduct = new PVector(0, 0, 0);
  private PVector crossProductDirectionRelativeToShoulder;
  private boolean crossProductMagnitudeWasAboveThreshold = false;
  private boolean crossProductMagnitudeIsAboveThreshold = false;
  private int activationTime; // millisseconds
  private int activationDirection; // [-3 ~ 3].
  
  RondDuBras(Skeleton skeleton, String whichHand){
    this.whichHand = whichHand;
    switch(this.whichHand){
      case "LEFT":
        this.handJoint = skeleton.joints[HAND_LEFT];
        this.shoulderJoint = skeleton.joints[SHOULDER_LEFT];
        break;
      case "RIGHT":
        this.handJoint = skeleton.joints[HAND_RIGHT];
        this.shoulderJoint = skeleton.joints[SHOULDER_RIGHT];
        break;
    }
    this.spineShoulderJoint = skeleton.joints[SPINE_SHOULDER];
  }
  
  /**
   * Updates the RondDuBras calculations.
   */
  private void update(){
    this.shoulderToHandPosition = PVector.sub(this.handJoint.estimatedPosition, this.shoulderJoint.estimatedPosition);
    this.shoulderToHandVelocity = PVector.sub(this.handJoint.estimatedVelocity, this.shoulderJoint.estimatedVelocity);
    this.currentCrossProduct = this.shoulderToHandPosition.cross(this.shoulderToHandVelocity);
    this.crossProductMagnitudeIsAboveThreshold = this.currentCrossProduct.mag() > this.crossProductMagnitudeThreshold;
    println("currentCrossProduct.mag(): " + this.currentCrossProduct.mag());
    
    if(this.crossProductMagnitudeIsAboveThreshold){
      this.crossProduct = this.currentCrossProduct;
    } else if(this.crossProductMagnitudeWasAboveThreshold){ // Pollock is activated here
      this.activationTime = millis();
      this.findDirection();
    }
    this.crossProductMagnitudeWasAboveThreshold = this.crossProductMagnitudeIsAboveThreshold;
  }
  
  /**
   * Find the crossProduct greatest direction.   
   */
  private void findDirection(){
    this.crossProductDirectionRelativeToShoulder = (new PVector(PVector.dot(this.crossProduct, this.spineShoulderJoint.estimatedDirectionX), 
                                                                PVector.dot(this.crossProduct, this.spineShoulderJoint.estimatedDirectionY), 
                                                                PVector.dot(this.crossProduct, this.spineShoulderJoint.estimatedDirectionZ))).normalize();
    float max = 0;
    if(abs(crossProductDirectionRelativeToShoulder.x) > max) {
      this.activationDirection = 1*sign(crossProductDirectionRelativeToShoulder.x);
      max = abs(crossProductDirectionRelativeToShoulder.x);
    }
    if(abs(crossProductDirectionRelativeToShoulder.y) > max) {
      this.activationDirection = 2*sign(crossProductDirectionRelativeToShoulder.y);
      max = abs(crossProductDirectionRelativeToShoulder.y);
    }
    if(abs(crossProductDirectionRelativeToShoulder.z) > max) {
      this.activationDirection = 3*sign(crossProductDirectionRelativeToShoulder.z);
      max = abs(crossProductDirectionRelativeToShoulder.z);
    }
    println("RondDuBrasActivationDirection: " + this.activationDirection);
  }
  
  /**
   * Draw the representations of the RondDuBras affordance.
   */
  private void draw(boolean drawCurrentCrossProduct, boolean drawPossibleDirectionsInTheOrigin){
    if(drawCurrentCrossProduct) this.drawCurrentCrossProduct(0.5);
    if(drawPossibleDirectionsInTheOrigin) this.drawPossibleDirectionsInTheOrigin(0.5);
    if(millis()-this.activationTime < this.fadeOutTime) this.drawActivationDirectionInTheOrigin(0.5);
  }
  
  /**
   * Draw the crossProduct starting from the shoulder joint. 
   */
  private void drawCurrentCrossProduct(float size){
    pushMatrix();
    strokeWeight(5);
    stroke(0, 0, 0, 255);
    translate(reScaleX(this.shoulderJoint.estimatedPosition.x, "RondDuBras.draw"),
              reScaleY(this.shoulderJoint.estimatedPosition.y, "RondDuBras.draw"),
              reScaleZ(this.shoulderJoint.estimatedPosition.z, "RondDuBras.draw"));
    line(0, 0, 0, size*reScaleX(this.currentCrossProduct.x, "RondDuBras.draw"), 
                  size*reScaleY(this.currentCrossProduct.y, "RondDuBras.draw"), 
                  size*reScaleZ(this.currentCrossProduct.z, "RondDuBras.draw"));
    popMatrix();
    println("crossProduct.mag(): " + crossProduct.mag());
  }
  
  /**
   * Draw the activated possible direction in the origin.
   */
  private void drawActivationDirectionInTheOrigin(float size){
    PVector directionActivated = new PVector(0, 0, 0);
         if(this.activationDirection == -3) directionActivated = new PVector( 0,  0, -1);
    else if(this.activationDirection == -2) directionActivated = new PVector( 0, -1,  0);
    else if(this.activationDirection == -1) directionActivated = new PVector(-1,  0,  0);
    else if(this.activationDirection ==  1) directionActivated = new PVector( 1,  0,  0);
    else if(this.activationDirection ==  2) directionActivated = new PVector( 0,  1,  0);
    else if(this.activationDirection ==  3) directionActivated = new PVector( 0,  0,  1);
    pushMatrix();
    strokeWeight(5);
    // Draw crossProductRelativeToShoulder:
    stroke(0, 0, 0, 128);
    line(0, 0, 0, size*reScaleX(this.crossProductDirectionRelativeToShoulder.x, "RondDuBras.draw"), 
                  size*reScaleY(this.crossProductDirectionRelativeToShoulder.y, "RondDuBras.draw"), 
                  size*reScaleZ(this.crossProductDirectionRelativeToShoulder.z, "RondDuBras.draw"));
    
    stroke(100, 0, 200, 255);
    // Draw possibleDirectionActivated:
    line(0, 0, 0, reScaleX(directionActivated.x, "RondDuBras.draw"), 
                  reScaleY(directionActivated.y, "RondDuBras.draw"), 
                  reScaleZ(directionActivated.z, "RondDuBras.draw"));
    
    // Draw shell:
    Quaternion azimuthClockwiseRotation =      axisAngleToQuaternion(0, 1, 0,  HALF_PI/2); // clockwise half rotation
    Quaternion azimuthAntiClockwiseRotation =  axisAngleToQuaternion(0, 1, 0, -HALF_PI/2); // anticlockwise half rotation
    Quaternion altitudeClockwiseRotation =     axisAngleToQuaternion(rotateVector(new PVector(directionActivated.x, 0, directionActivated.z), new PVector(0, 1, 0), HALF_PI),  HALF_PI/2); // clockwise half rotation
    Quaternion altitudeAntiClockwiseRotation = axisAngleToQuaternion(rotateVector(new PVector(directionActivated.x, 0, directionActivated.z), new PVector(0, 1, 0), HALF_PI), -HALF_PI/2); // anticlockwise half rotation
    
    PVector vertex1 = rotateVector(rotateVector(directionActivated, altitudeClockwiseRotation)    , azimuthClockwiseRotation);
    PVector vertex2 = rotateVector(rotateVector(directionActivated, altitudeAntiClockwiseRotation), azimuthClockwiseRotation);
    PVector vertex3 = rotateVector(rotateVector(directionActivated, altitudeAntiClockwiseRotation), azimuthAntiClockwiseRotation);
    PVector vertex4 = rotateVector(rotateVector(directionActivated, altitudeClockwiseRotation)    , azimuthAntiClockwiseRotation);
    
    float fadeOutFactor = 1-(float)((millis()-this.activationTime)/(float)this.fadeOutTime);
    fill(100, 0, 200, 255*fadeOutFactor);
    beginShape();
    vertex(vertex1, "RondDuBras.draw");
    vertex(vertex2, "RondDuBras.draw");
    vertex(vertex3, "RondDuBras.draw");
    vertex(vertex4, "RondDuBras.draw");
    endShape(CLOSE);
    popMatrix();
  }
  
  /**
   * Draw the possible directions in the origin.
   */
  private void drawPossibleDirectionsInTheOrigin(float size){
    pushMatrix();
    strokeWeight(2);
    stroke(0, 0, 0, 40);
    noFill();

    PVector vertex1 = (new PVector(-1, -1, -1)).mult(size);
    PVector vertex2 = (new PVector(-1, -1,  1)).mult(size);
    PVector vertex3 = (new PVector(-1,  1,  1)).mult(size);
    PVector vertex4 = (new PVector(-1,  1, -1)).mult(size);
    PVector vertex5 = (new PVector( 1, -1, -1)).mult(size);
    PVector vertex6 = (new PVector( 1, -1,  1)).mult(size);
    PVector vertex7 = (new PVector( 1,  1,  1)).mult(size);
    PVector vertex8 = (new PVector( 1,  1, -1)).mult(size);
    
    // Left Side Face:
    beginShape();
    vertex(vertex1, "RondDuBras.draw");
    vertex(vertex2, "RondDuBras.draw");
    vertex(vertex3, "RondDuBras.draw");
    vertex(vertex4, "RondDuBras.draw");
    endShape(CLOSE);
    
    // Right Side Face:
    beginShape();
    vertex(vertex5, "RondDuBras.draw");
    vertex(vertex6, "RondDuBras.draw");
    vertex(vertex7, "RondDuBras.draw");
    vertex(vertex8, "RondDuBras.draw");
    endShape(CLOSE);
    
    // Lines Connecting Faces:
    beginShape(LINES);
    vertex(vertex1, "RondDuBras.draw");
    vertex(vertex5, "RondDuBras.draw");
    vertex(vertex2, "RondDuBras.draw");
    vertex(vertex6, "RondDuBras.draw");
    vertex(vertex3, "RondDuBras.draw");
    vertex(vertex7, "RondDuBras.draw");
    vertex(vertex4, "RondDuBras.draw");
    vertex(vertex8, "RondDuBras.draw");
    endShape();
    popMatrix();
  }
}
