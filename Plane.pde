class Plane{
  private float a, b, c, d; // Plane equation: a*x + b*y + c*z = d
  private PVector basisVector1;
  private PVector basisVector2;
  private PVector middlePoint;
  private PVector normalVector;
  
  public Plane(PVector basisVector1, PVector basisVector2, PVector middlePoint){
    this.basisVector1 = basisVector1;
    this.basisVector2 = basisVector2;
    this.normalVector = this.basisVector1.cross(this.basisVector2).normalize();
    this.middlePoint = middlePoint;
    this.a = this.normalVector.x;
    this.b = this.normalVector.y;
    this.c = this.normalVector.z;
    this.d = -PVector.dot(this.normalVector, this.middlePoint);
  }
  
  public float distanceTo(PVector point){
    return abs(this.a*point.x + this.b*point.y + this.c*point.z + this.d);
  }
  
  public void draw(float dimension){
    PVector floorCorner1 = PVector.add(this.middlePoint, PVector.mult(this.basisVector1, dimension)).add(PVector.mult(this.basisVector2, dimension));
    PVector floorCorner2 = PVector.add(this.middlePoint, PVector.mult(this.basisVector1, -dimension)).add(PVector.mult(this.basisVector2, dimension));
    PVector floorCorner3 = PVector.add(this.middlePoint, PVector.mult(this.basisVector1, -dimension)).add(PVector.mult(this.basisVector2, -dimension));
    PVector floorCorner4 = PVector.add(this.middlePoint, PVector.mult(this.basisVector1, dimension)).add(PVector.mult(this.basisVector2, -dimension));

    fill(90, 30, 90, 50);
    beginShape();
    vertex(reScaleX(floorCorner1.x), reScaleY(floorCorner1.y), reScaleZ(floorCorner1.z));
    vertex(reScaleX(floorCorner2.x), reScaleY(floorCorner2.y), reScaleZ(floorCorner2.z));
    vertex(reScaleX(floorCorner3.x), reScaleY(floorCorner3.y), reScaleZ(floorCorner3.z));
    vertex(reScaleX(floorCorner4.x), reScaleY(floorCorner4.y), reScaleZ(floorCorner4.z));
    endShape(CLOSE);
  }
}
