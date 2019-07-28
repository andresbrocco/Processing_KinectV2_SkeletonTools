class Momentum{
  private Skeleton skeleton;
  private final float smoothStep = 0.15; //
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
    this.fluid = 0;
    this.harsh = 0;
    this.total = 0;
    for (int j = 0; j < 25; j++){
      if(this.skeleton.joints[j].estimatedAcceleration.mag() > 0.0001){ // prevent from falling into NaN
        PVector previousVelocityDirection = PVector.sub(this.skeleton.joints[j].estimatedVelocity, this.skeleton.joints[j].estimatedAcceleration).normalize();
        float tangentialAccelerationMagnitude = this.skeleton.joints[j].estimatedVelocity.cross(previousVelocityDirection).mag();
        float radialAccelerationMagnitude = abs(PVector.dot(this.skeleton.joints[j].estimatedVelocity, previousVelocityDirection));
        float tangentialAccelerationPercentage = tangentialAccelerationMagnitude/(tangentialAccelerationMagnitude+radialAccelerationMagnitude);
        float radialAccelerationPercentage = radialAccelerationMagnitude/(tangentialAccelerationMagnitude+radialAccelerationMagnitude);
        this.fluid += tangentialAccelerationPercentage/25;
        this.harsh += radialAccelerationPercentage/25;
        this.total += this.skeleton.joints[j].estimatedAcceleration.mag();
      }
    }
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
    //text("Total Momentum", 200, 0);
    //rect(100, 0, 200, 20);
  }
}
