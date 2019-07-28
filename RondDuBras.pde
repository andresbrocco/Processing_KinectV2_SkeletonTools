/**
 * RondDuBras: Trigger Affordance that quantifies the "round" movement of each hand.
 * TODO: Finish Javadoc Documentation.
 */
public class RondDuBras{
  private final float crossProductMagnitudeThreshold = 0.5;
  private final float minimumMovementAmplitude = QUARTER_PI; // arc length (radians). to be adjusted
  private int fadeOutTime = 1000; // millisseconds
  private String whichHand; // LEFT or RIGHT
  private Joint handJoint;
  private Joint shoulderJoint;
  private Joint spineShoulderJoint;
  private PVector[] spineShoulderOrientationWhenCrossedUpTheThreshold = new PVector[3];
  private PVector[] spineShoulderOrientationWhenCrossedDownTheThreshold = new PVector[3];
  private PVector[] averageSpineShoulderOrientationWhenAboveThreshold = new PVector[3];
  private PVector currentShoulderToHandPosition;
  private PVector shoulderToHandPositionWhenCrossedUpTheThreshold;
  private PVector shoulderToHandPositionWhenCrossedDownTheThreshold;
  private PVector currentShoulderToHandDirection;
  private PVector currentShoulderToHandVelocity = new PVector(0, 0, 0);
  private float   currentShoulderToHandRadialSpeed = 0;
  private float   currentShoulderToHandTangentialSpeed = 0;
  private PVector currentCrossProduct = new PVector(0, 0, 0);
  private float   currentCrossProductMagnitude = 0; // adjusted by perpendicularismFactor
  private PVector averageCrossProductWhenAboveThreshold = new PVector(0, 0, 0);
  private int numberOfFramesAboveThreshold = 0;
  private PVector crossProductDirectionWhenActivatedRelativeToSpineShoulder;
  private boolean previousCrossProductMagnitudeWasAboveThreshold = false;
  private boolean currentCrossProductMagnitudeIsAboveThreshold = false;
  private int timeWhenActivated; // millisseconds
  private PVector activatedDirection;
  private PVector[] activatedDirectionSextantVertexes = new PVector[4];
  private int activatedDirectionCode; // [-3 ~ 3].
  
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
    this.currentShoulderToHandPosition = PVector.sub(this.handJoint.estimatedPosition, this.shoulderJoint.estimatedPosition);
    this.currentShoulderToHandDirection = PVector.div(this.currentShoulderToHandPosition, this.currentShoulderToHandPosition.mag());
    this.currentShoulderToHandVelocity = PVector.sub(this.handJoint.estimatedVelocity, this.shoulderJoint.estimatedVelocity);
    this.currentShoulderToHandRadialSpeed = PVector.dot(this.currentShoulderToHandVelocity, this.currentShoulderToHandDirection);
    this.currentShoulderToHandTangentialSpeed = this.currentShoulderToHandVelocity.cross(this.currentShoulderToHandDirection).mag();
    this.currentCrossProduct = this.currentShoulderToHandPosition.cross(this.currentShoulderToHandVelocity);
    float perpendicularismFactor = abs(currentShoulderToHandTangentialSpeed)/this.currentShoulderToHandVelocity.mag();
    //println("perpendicularismFactor: " + perpendicularismFactor);
    this.currentCrossProductMagnitude = this.currentCrossProduct.mag()*perpendicularismFactor;
    //println("currentCrossProductMagnitude: " + this.currentCrossProductMagnitude);
    this.currentCrossProductMagnitudeIsAboveThreshold = this.currentCrossProductMagnitude > this.crossProductMagnitudeThreshold;
    
    if(this.currentCrossProductMagnitudeIsAboveThreshold){
      if(this.previousCrossProductMagnitudeWasAboveThreshold){ // Remained above the threshold.
        this.numberOfFramesAboveThreshold++;
        float step = 1/(float)this.numberOfFramesAboveThreshold;
        this.averageCrossProductWhenAboveThreshold = slerp(this.averageCrossProductWhenAboveThreshold, this.currentCrossProduct, step);
        this.averageSpineShoulderOrientationWhenAboveThreshold[0] = slerp(this.averageSpineShoulderOrientationWhenAboveThreshold[0], this.spineShoulderJoint.estimatedDirectionX, step); //step);
        this.averageSpineShoulderOrientationWhenAboveThreshold[1] = slerp(this.averageSpineShoulderOrientationWhenAboveThreshold[1], this.spineShoulderJoint.estimatedDirectionY, step); //step);
        this.averageSpineShoulderOrientationWhenAboveThreshold[2] = slerp(this.averageSpineShoulderOrientationWhenAboveThreshold[2], this.spineShoulderJoint.estimatedDirectionZ, step); //step);
      } else { // Crossed up the threshold.
        this.numberOfFramesAboveThreshold = 1;
        this.averageCrossProductWhenAboveThreshold = this.currentCrossProduct;
        this.averageSpineShoulderOrientationWhenAboveThreshold[0] = this.spineShoulderJoint.estimatedDirectionX;
        this.averageSpineShoulderOrientationWhenAboveThreshold[1] = this.spineShoulderJoint.estimatedDirectionY;
        this.averageSpineShoulderOrientationWhenAboveThreshold[2] = this.spineShoulderJoint.estimatedDirectionZ;
        this.shoulderToHandPositionWhenCrossedUpTheThreshold = this.currentShoulderToHandPosition;
        this.spineShoulderOrientationWhenCrossedUpTheThreshold[0] = this.spineShoulderJoint.estimatedDirectionX;
        this.spineShoulderOrientationWhenCrossedUpTheThreshold[1] = this.spineShoulderJoint.estimatedDirectionY;
        this.spineShoulderOrientationWhenCrossedUpTheThreshold[2] = this.spineShoulderJoint.estimatedDirectionZ;
      }
    } else if(this.previousCrossProductMagnitudeWasAboveThreshold){ // Crossed down the threshold. 
      this.shoulderToHandPositionWhenCrossedDownTheThreshold = this.currentShoulderToHandPosition;
      this.spineShoulderOrientationWhenCrossedDownTheThreshold[0] = this.spineShoulderJoint.estimatedDirectionX;
      this.spineShoulderOrientationWhenCrossedDownTheThreshold[1] = this.spineShoulderJoint.estimatedDirectionY;
      this.spineShoulderOrientationWhenCrossedDownTheThreshold[2] = this.spineShoulderJoint.estimatedDirectionZ;
      
      float movementAmplitude = PVector.angleBetween(this.shoulderToHandPositionWhenCrossedUpTheThreshold, this.shoulderToHandPositionWhenCrossedDownTheThreshold);
      //println("movementAmplitude: " + movementAmplitude);
      if(movementAmplitude > this.minimumMovementAmplitude){
        this.timeWhenActivated = millis();
        this.findDirection();
      }
    }
    this.previousCrossProductMagnitudeWasAboveThreshold = this.currentCrossProductMagnitudeIsAboveThreshold;
  }
  
  /**
   * Find the crossProduct greatest direction.   
   */
  private void findDirection(){
    this.crossProductDirectionWhenActivatedRelativeToSpineShoulder = (new PVector(PVector.dot(this.averageCrossProductWhenAboveThreshold, this.averageSpineShoulderOrientationWhenAboveThreshold[0]), 
                                                                                  PVector.dot(this.averageCrossProductWhenAboveThreshold, this.averageSpineShoulderOrientationWhenAboveThreshold[1]), 
                                                                                  PVector.dot(this.averageCrossProductWhenAboveThreshold, this.averageSpineShoulderOrientationWhenAboveThreshold[2]))).normalize();
    float max = 0;
    if(crossProductDirectionWhenActivatedRelativeToSpineShoulder.x > max) {
      this.activatedDirectionCode = 1;
      this.activatedDirection = new PVector(1,  0,  0);
      this.activatedDirectionSextantVertexes[0] = new PVector(1, -1, -1);
      this.activatedDirectionSextantVertexes[1] = new PVector(1, -1,  1);
      this.activatedDirectionSextantVertexes[2] = new PVector(1,  1,  1);
      this.activatedDirectionSextantVertexes[3] = new PVector(1,  1, -1);
      max = crossProductDirectionWhenActivatedRelativeToSpineShoulder.x;
    }
    if(-crossProductDirectionWhenActivatedRelativeToSpineShoulder.x > max) {
      this.activatedDirectionCode = -1;
      this.activatedDirection = new PVector(-1,  0,  0);
      this.activatedDirectionSextantVertexes[0] = new PVector(-1, -1, -1);
      this.activatedDirectionSextantVertexes[1] = new PVector(-1, -1,  1);
      this.activatedDirectionSextantVertexes[2] = new PVector(-1,  1,  1);
      this.activatedDirectionSextantVertexes[3] = new PVector(-1,  1, -1);
      max = -crossProductDirectionWhenActivatedRelativeToSpineShoulder.x;
    }
    if(crossProductDirectionWhenActivatedRelativeToSpineShoulder.y > max) {
      this.activatedDirectionCode = 2;
      this.activatedDirection = new PVector(0,  1,  0);
      this.activatedDirectionSextantVertexes[0] = new PVector(-1, 1, -1);
      this.activatedDirectionSextantVertexes[1] = new PVector(-1, 1,  1);
      this.activatedDirectionSextantVertexes[2] = new PVector( 1, 1,  1);
      this.activatedDirectionSextantVertexes[3] = new PVector( 1, 1, -1);
      max = crossProductDirectionWhenActivatedRelativeToSpineShoulder.y;
    }
    if(-crossProductDirectionWhenActivatedRelativeToSpineShoulder.y > max) {
      this.activatedDirectionCode = -2;
      this.activatedDirection = new PVector(0, -1,  0);
      this.activatedDirectionSextantVertexes[0] = new PVector(-1, -1, -1);
      this.activatedDirectionSextantVertexes[1] = new PVector(-1, -1,  1);
      this.activatedDirectionSextantVertexes[2] = new PVector( 1, -1,  1);
      this.activatedDirectionSextantVertexes[3] = new PVector( 1, -1, -1);
      max = -crossProductDirectionWhenActivatedRelativeToSpineShoulder.y;
    }
    if(crossProductDirectionWhenActivatedRelativeToSpineShoulder.z > max) {
      this.activatedDirectionCode = 3;
      this.activatedDirection = new PVector(0, 0,  1);
      this.activatedDirectionSextantVertexes[0] = new PVector(-1, -1, 1);
      this.activatedDirectionSextantVertexes[1] = new PVector(-1,  1, 1);
      this.activatedDirectionSextantVertexes[2] = new PVector( 1,  1, 1);
      this.activatedDirectionSextantVertexes[3] = new PVector( 1, -1, 1);
      max = crossProductDirectionWhenActivatedRelativeToSpineShoulder.z;
    }
    if(-crossProductDirectionWhenActivatedRelativeToSpineShoulder.z > max) {
      this.activatedDirectionCode = -3;
      this.activatedDirection = new PVector(0, 0, -1);
      this.activatedDirectionSextantVertexes[0] = new PVector(-1, -1, -1);
      this.activatedDirectionSextantVertexes[1] = new PVector(-1,  1, -1);
      this.activatedDirectionSextantVertexes[2] = new PVector( 1,  1, -1);
      this.activatedDirectionSextantVertexes[3] = new PVector( 1, -1, -1);
      max = -crossProductDirectionWhenActivatedRelativeToSpineShoulder.z;
    }
    //println("RondDuBrasActivatedDirectionCode: " + this.activatedDirectionCode);
  }
  
  /**
   * Draw the representations of the RondDuBras affordance.
   */
  private void draw(boolean drawCurrentCrossProduct, boolean drawPossibleDirectionsInTheOrigin){
    if(drawCurrentCrossProduct) this.drawCurrentCrossProduct(0.5);
    if(drawPossibleDirectionsInTheOrigin) this.drawPossibleDirectionsInTheOrigin(0.3);
    if(millis()-this.timeWhenActivated < this.fadeOutTime) this.drawActivatedDirectionInTheOrigin(0.3);
  }
  
  /**
   * Draw the crossProduct starting from the shoulder joint. 
   */
  private void drawCurrentCrossProduct(float size){
    pushMatrix();
    strokeWeight(5);
    stroke(0, 0, 0, 255*min(1, currentCrossProductMagnitude));
    fill(0, 0, 0, 255*min(1, currentCrossProductMagnitude));
    translate(reScaleX(this.shoulderJoint.estimatedPosition.x, "RondDuBras.draw"),
              reScaleY(this.shoulderJoint.estimatedPosition.y, "RondDuBras.draw"),
              reScaleZ(this.shoulderJoint.estimatedPosition.z, "RondDuBras.draw"));
    if(this.currentCrossProductMagnitudeIsAboveThreshold){
      line(0, 0, 0, size*reScaleX(this.averageCrossProductWhenAboveThreshold.x, "RondDuBras.draw"), 
                    size*reScaleY(this.averageCrossProductWhenAboveThreshold.y, "RondDuBras.draw"), 
                    size*reScaleZ(this.averageCrossProductWhenAboveThreshold.z, "RondDuBras.draw"));
      drawPie3D(this.shoulderToHandPositionWhenCrossedUpTheThreshold, this.currentShoulderToHandPosition, size);
    } else if(millis()-this.timeWhenActivated < this.fadeOutTime){
      line(0, 0, 0, size*reScaleX(this.averageCrossProductWhenAboveThreshold.x, "RondDuBras.draw"), 
                    size*reScaleY(this.averageCrossProductWhenAboveThreshold.y, "RondDuBras.draw"), 
                    size*reScaleZ(this.averageCrossProductWhenAboveThreshold.z, "RondDuBras.draw"));
      float fadeOutFactor = 1-(float)((millis()-this.timeWhenActivated)/(float)this.fadeOutTime);
      fill(0, 0, 0, 255*fadeOutFactor);
      drawPie3D(this.shoulderToHandPositionWhenCrossedUpTheThreshold, this.shoulderToHandPositionWhenCrossedDownTheThreshold, size);
    }
    popMatrix();
  }
  
  /**
   * Draw the activated possible direction in the origin.
   */
  private void drawActivatedDirectionInTheOrigin(float size){
    pushMatrix();
    strokeWeight(5);
    
    // Draw crossProductDirectionWhenActivatedRelativeToSpineShoulder:
    stroke(0, 0, 0, 128);
    line(0, 0, 0, reScaleX(this.crossProductDirectionWhenActivatedRelativeToSpineShoulder.x, "RondDuBras.draw")*size*3/sqrt(3), 
                  reScaleY(this.crossProductDirectionWhenActivatedRelativeToSpineShoulder.y, "RondDuBras.draw")*size*3/sqrt(3), 
                  reScaleZ(this.crossProductDirectionWhenActivatedRelativeToSpineShoulder.z, "RondDuBras.draw")*size*3/sqrt(3));
    
    // Draw activatedDirection:
    stroke(100, 0, 200, 255);
    line(0, 0, 0, reScaleX(this.activatedDirection.x, "RondDuBras.draw")*size*3/sqrt(3), 
                  reScaleY(this.activatedDirection.y, "RondDuBras.draw")*size*3/sqrt(3), 
                  reScaleZ(this.activatedDirection.z, "RondDuBras.draw")*size*3/sqrt(3));
    
    // Draw shell:
    float fadeOutFactor = 1-(float)((millis()-this.timeWhenActivated)/(float)this.fadeOutTime);
    fill(100, 0, 200, 255*fadeOutFactor);
    beginShape();
    vertex(PVector.mult(this.activatedDirectionSextantVertexes[0], size), "RondDuBras.draw");
    vertex(PVector.mult(this.activatedDirectionSextantVertexes[1], size), "RondDuBras.draw");
    vertex(PVector.mult(this.activatedDirectionSextantVertexes[2], size), "RondDuBras.draw");
    vertex(PVector.mult(this.activatedDirectionSextantVertexes[3], size), "RondDuBras.draw");
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
