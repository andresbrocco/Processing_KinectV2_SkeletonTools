import Jama.*; // Java Matrix Library: https://math.nist.gov/javanumerics/jama/

public class Quaternion{
  private float real;
  private PVector vector;
  
  public Quaternion(){
    this.real = 0;
    this.vector = new PVector(0,0,0);
  }
  public Quaternion(KQuaternion kQuaternion){
    this.real = kQuaternion.getW();
    this.vector = new PVector(kQuaternion.getX(), kQuaternion.getY(), kQuaternion.getZ());
  }
  
  public Quaternion(float w, float x, float y, float z){
    this.real = w;
    this.vector = new PVector(x, y, z);
  }
  
  public Quaternion getCopy(){
    Quaternion qCopy = new Quaternion(0,0,0,0);
    qCopy.real = this.real;
    qCopy.vector.x = this.vector.x;
    qCopy.vector.y = this.vector.y;
    qCopy.vector.z = this.vector.z;
    return qCopy;
  }
  
  public float mag(){
    return sqrt(pow(this.real, 2)+pow(this.vector.x, 2)+pow(this.vector.y, 2)+pow(this.vector.z, 2));
  }
  
  public void normalize(){
    if(this.mag() < 0.0001){ // avoid division by zero.
      this.real = 1;
      this.vector = new PVector(0,0,0);
    }
    else {
      this.real = this.real/this.mag();
      this.vector = PVector.div(this.vector, this.mag());
    }
  }
}

Quaternion qConjugate(Quaternion q){
  Quaternion qConj = new Quaternion(0,0,0,0);
  qConj.real = q.real;
  qConj.vector = PVector.mult(q.vector, -1);
  return qConj;
}

Quaternion qSlerp(Quaternion q1, Quaternion q2, float step){ 
  q1.normalize();
  q2.normalize();
  Quaternion q2_ = q2.getCopy();
  float dotProduct = q1.real*q2.real + PVector.dot(q1.vector, q2.vector);
  if(dotProduct < 0){
    q2_.real = -q2_.real;
    q2_.vector = PVector.mult(q2_.vector, -1);
    dotProduct = -dotProduct;
  }
  Quaternion qRes = new Quaternion(0, 0, 0, 0);
  if(dotProduct > 0.9995){ // If quaternions are too close, linearly interpolate and normalize the result.
    qRes.real = lerp(q1.real, q2_.real, step);
    qRes.vector = PVector.lerp(q1.vector, q2_.vector, step);
    qRes.normalize();
    return qRes;
  } 
  else { // Since dot is in range [0, 0.9995], acos() is safe;
    float theta_0 = acos(dotProduct);        // theta_0 = angle between input vectors
    float theta = theta_0*step;              // theta = angle between v0 and result
    float q1Multiplier = sin(theta_0-theta)/sin(theta_0);
    float q2Multiplier = sin(theta)/sin(theta_0);
    qRes.real = q1.real*q1Multiplier + q2_.real*q2Multiplier;
    qRes.vector = PVector.add(PVector.mult(q1.vector, q1Multiplier), PVector.mult(q2_.vector, q2Multiplier));
    return qRes;
  }
}

Quaternion qMult(Quaternion q1, Quaternion q2){
  Quaternion qRes = new Quaternion(0, 0, 0, 0);
  qRes.real = q1.real*q2.real - PVector.dot(q1.vector, q2.vector);
  qRes.vector = PVector.add(PVector.mult(q2.vector, q1.real), PVector.mult(q1.vector, q2.real)).add(q1.vector.cross(q2.vector));
  return qRes;
}

PVector quaternionToEulerAngles(Quaternion q){
  PVector eulerAngles = new PVector(0,0,0);
  eulerAngles.x = atan2(2*(q.real*q.vector.x + q.vector.y*q.vector.z), 1-2*(q.vector.x*q.vector.x+q.vector.y*q.vector.y));
  eulerAngles.y = asin(2*(q.real*q.vector.y - q.vector.x*q.vector.z));
  eulerAngles.x = atan2(2*(q.real*q.vector.z + q.vector.x*q.vector.y), 1-2*(q.vector.y*q.vector.y+q.vector.z*q.vector.z));
  return eulerAngles;
}

Quaternion rotationMatrixToQuaternion(Matrix rotationMatrix){
  println("determinant of rotationMatrix: "+ rotationMatrix.det()); // must be 1. if -1, one basisVector should have been inverted.
  PVector rotationAxis = new PVector();
  EigenvalueDecomposition eigenvalueDecomposition = rotationMatrix.eig();
  double[] eigenvaluesReal = eigenvalueDecomposition.getRealEigenvalues();
  double[] eigenvaluesImag = eigenvalueDecomposition.getImagEigenvalues();
  Matrix eigenvectors = eigenvalueDecomposition.getV();
  for(int eig=0; eig<3; eig++){
    float tol = 0.01; // numerical tolerance
    if(eigenvaluesReal[eig]>1-tol && eigenvaluesReal[eig]<1+tol && eigenvaluesImag[eig]<tol && eigenvaluesImag[eig]>-tol){
      rotationAxis = new PVector((float)eigenvectors.get(0, eig), (float)eigenvectors.get(1, eig), (float)eigenvectors.get(2, eig));
      println("FOUND eigenvalue: "+eigenvaluesReal[eig] +"+i*"+eigenvaluesImag[eig]+ " with eigenvector: "+rotationAxis.x + " "+ rotationAxis.y + " "+rotationAxis.z);
      break;
    }
    else{
      println("wrong eigenvalue: "+eigenvaluesReal[eig] +"+i*"+eigenvaluesImag[eig]);
    }
  }
  float cosTheta = (float)(rotationMatrix.trace()-1)/2;
  if(cosTheta > 1){
    println("warning: cosTheta > 1. Truncating to 1");
    cosTheta = 1;
  } else if(cosTheta < -1){
    println("warning: cosTheta < -1. Truncating to -1");
    cosTheta = -1;
  }
  float theta = acos(cosTheta);
  println("theta = "+ theta);
  Quaternion quaternion = new Quaternion();
  quaternion.real = cos(theta/2);
  quaternion.vector = PVector.mult(rotationAxis, sin(theta/2));
  println("quaternion: "+quaternion.real+" "+quaternion.vector+ ". With norm: "+ quaternion.mag());
  return quaternion;
}

float angleBetweenQuaternions(Quaternion q1, Quaternion q2){
  q1.normalize();
  q2.normalize();
  float dotProduct = q1.real*q2.real + PVector.dot(q1.vector, q2.vector);
  if(dotProduct < 0){
    dotProduct = -dotProduct;
  }
  float theta = acos(min(dotProduct, 1));
  return theta;
}