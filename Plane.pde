class Plane{
  public float a, b, c, d; // Plane equation: a*x + b*y + c*z = d
  public PVector middlePoint;
  public PVector normalVector;
  
  public Plane(PVector basisVector1, PVector basisVector2, PVector middlePoint){
    this.normalVector = basisVector1.cross(basisVector2);
    this.normalVector.normalize();
    this.middlePoint = middlePoint;
    this.a = this.normalVector.x;
    this.b = this.normalVector.y;
    this.c = this.normalVector.z;
    this.d = -PVector.dot(this.normalVector, middlePoint);
  }
  
  public float distanceTo(PVector point){
    return abs(this.a*point.x + this.b*point.y + this.c*point.z + this.d);
  }
}
