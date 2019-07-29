class Momentum{
  private Skeleton skeleton;
  private final float smoothStep = 0.3; //
  public float fluid = 0; // in percentage. 
  public float harsh = 0; // in percentage. 
  public float total = 0; // in m/s². Generally, below 10 can be considered no intention of movement, thus resting state.
  public float averageFluid = 0;
  public float averageHarsh = 0;
  public float averageTotal = 0;
  
  Momentum(Skeleton skeleton){
    this.skeleton = skeleton;
  }
  
  public void update(){
    float averageTangentialAccelerationMagnitude = 0;
    float averageRadialAccelerationMagnitude = 0;
    this.total = 0;
    for (int j = 0; j < 25; j++){
      PVector previousVelocityDirection = PVector.sub(this.skeleton.joints[j].estimatedVelocity, this.skeleton.joints[j].estimatedAcceleration).normalize();
      float tangentialAccelerationMagnitude = this.skeleton.joints[j].estimatedVelocity.cross(previousVelocityDirection).mag();
      float radialAccelerationMagnitude = abs(PVector.dot(this.skeleton.joints[j].estimatedVelocity, previousVelocityDirection));
      averageTangentialAccelerationMagnitude += tangentialAccelerationMagnitude/25;
      averageRadialAccelerationMagnitude += radialAccelerationMagnitude/25;
      this.total += this.skeleton.joints[j].estimatedAcceleration.mag();
    }
    if(averageTangentialAccelerationMagnitude+averageRadialAccelerationMagnitude> 0.0001){ // prevent from falling into NaN
      this.fluid = averageTangentialAccelerationMagnitude/(averageTangentialAccelerationMagnitude+averageRadialAccelerationMagnitude);
      this.harsh = averageRadialAccelerationMagnitude/(averageTangentialAccelerationMagnitude+averageRadialAccelerationMagnitude);
    }
    println("fluid: "+this.fluid+" harsh: "+this.harsh);
    this.averageFluid = lerp(this.averageFluid, this.fluid, this.smoothStep);
    this.averageHarsh = lerp(this.averageHarsh, this.harsh, this.smoothStep);
    this.averageTotal = lerp(this.averageTotal, this.total, this.smoothStep);
    /*
    println("this.fluid: "+ this.averageFluid);
    println("this.harsh: "+ this.averageHarsh);
    println("this.total: "+ this.averageTotal);
    */
  }
  
  public void draw(){ // Draw 2 bars: one indicating percentage of harsh/fluid and another indicating total.
  // Da pra ver que tem algo errado no sistema de coordenadas que eu to usando, desde o inicio... Eu considerei que o eixo y é pra cima, enquanto ele é pra baixo. Isso explica varias coisas... não sei onde devo começar pra arrumar isso haha
    float offsetX = -2; // meters
    float textHeight = 0.1; // meters
    float rectHeight = textHeight; // meters
    float rectLength = 1; // meters
    strokeWeight(3);
    pushMatrix();
    rotateX(PI);
    textSize(reScaleY(textHeight, "momentum.draw"));
    
    // Total Momentum:
    stroke(123, 88, 31);
    fill(123, 88, 31);
    text("Total Momentum", reScaleX(offsetX, "momentum.draw"), 0);
    float totalMomentumPercentage = min(1, map(this.averageTotal, 0, 200, 0, 1));
    fill(123, 88, 31, 128);
    rect(reScaleX(offsetX, "momentum.draw"), 0, totalMomentumPercentage*reScaleX(rectLength, "momentum.draw"), reScaleY(rectHeight, "momentum.draw"));
    noFill();
    rect(reScaleX(offsetX, "momentum.draw"), 0, reScaleX(rectLength, "momentum.draw"), reScaleY(rectHeight, "momentum.draw"));
    
    // Fluid/Harsh percentage:
    stroke(150, 25, 170);
    fill(150, 25, 170);
    text("Fluid       /      Harsh", reScaleX(offsetX, "momentum.draw"), reScaleY(rectHeight+textHeight, "momentum.draw"));
    fill(150, 25, 170, 128);
    rect(reScaleX(offsetX, "momentum.draw"), reScaleY(rectHeight+textHeight, "momentum.draw"), this.averageFluid*reScaleX(rectLength, "momentum.draw"), reScaleY(rectHeight, "momentum.draw"));
    noFill();
    rect(reScaleX(offsetX, "momentum.draw"), reScaleY(rectHeight+textHeight, "momentum.draw"), reScaleX(rectLength, "momentum.draw"), reScaleY(rectHeight, "momentum.draw"));
    
    popMatrix();
  }
}
