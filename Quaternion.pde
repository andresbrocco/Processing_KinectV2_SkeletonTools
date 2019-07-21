import Jama.*; // Java Matrix Library: https://math.nist.gov/javanumerics/jama/

/**
 * This class implements some useful quaternion operations.
 */
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
  
  public Quaternion(float real, PVector vector){
    this.real = real;
    this.vector = vector;
  }
  
/**
 * @return a copy instead of a pointer to the original object.
 */
  public Quaternion getCopy(){
    Quaternion qCopy = new Quaternion(0,0,0,0);
    qCopy.real = this.real;
    qCopy.vector.x = this.vector.x;
    qCopy.vector.y = this.vector.y;
    qCopy.vector.z = this.vector.z;
    return qCopy;
  }
  
/**
 * @return magnitude of the Quaternion.
 */
  public float mag(){
    return sqrt(pow(this.real, 2)+pow(this.vector.x, 2)+pow(this.vector.y, 2)+pow(this.vector.z, 2));
  }
  
/**
 * Normalize the Quaternion.
 */
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
  
/**
 * Get the conjugate of a quaternion
 * @param q quaternion to be conjugated
 * @return conjugate of q
 */
Quaternion qConjugate(Quaternion q){
  Quaternion qConj = new Quaternion(0,0,0,0);
  qConj.real = q.real;
  qConj.vector = PVector.mult(q.vector, -1);
  return qConj;
}

/**
 * Spherical Linear Interpolation between quaternions.
 * @param q1 first quaternion
 * @param q2 second quaternion
 * @param step percentage of path between q1 and q2. (or beyond, if you want)
 * @return interpolation/extrapolation from q1 to q2.
 */
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

/**
 * Efficient quaternion multiplication.
 * @param q1 first quaternion.
 * @param q2 second quaternion.
 * @return q1*q2 (using special quaternion algebra).
 */
Quaternion qMult(Quaternion q1, Quaternion q2){
  Quaternion qRes = new Quaternion(0, 0, 0, 0);
  qRes.real = q1.real*q2.real - PVector.dot(q1.vector, q2.vector);
  qRes.vector = PVector.add(PVector.mult(q2.vector, q1.real), PVector.mult(q1.vector, q2.real)).add(q1.vector.cross(q2.vector));
  return qRes;
}

/**
 * Convert from quaternion representation of orientation to Euler Angles. (intrinsic Tait-Bryan angles following z-y'-x'').
 * @param q1 quaternion representing orientation.
 * @return PVector of euler angles (roll, pitch, yaw) in radians.
 */
PVector quaternionToEulerAngles(Quaternion q){
  PVector eulerAngles = new PVector(0,0,0);
  eulerAngles.x = atan2(2*(q.real*q.vector.x + q.vector.y*q.vector.z), 1-2*(q.vector.x*q.vector.x+q.vector.y*q.vector.y));
  eulerAngles.y = asin(2*(q.real*q.vector.y - q.vector.x*q.vector.z));
  eulerAngles.x = atan2(2*(q.real*q.vector.z + q.vector.x*q.vector.y), 1-2*(q.vector.y*q.vector.y+q.vector.z*q.vector.z));
  return eulerAngles;
}

/**
 * Get the minimum angle between two quaternions.
 * @param q1 quaternion 1.
 * @param q2 quaternion 2.
 * @return angle in radians (0<theta<PI).
 */
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

/**
 * Convert from axis-angle to Quaternion representation of rotation.
 * @param angle rotation around the vector (x, y, z).
 * @param x
 * @param y
 * @param z
 * @return quaternion.
 */
Quaternion axisAngleToQuaternion(float x, float y, float z, float angle){
  return axisAngleToQuaternion(new PVector(x, y, z), angle);
}

/**
 * Convert from axis-angle to Quaternion representation of rotation.
 * @param angle rotation around the axis vector (x, y, z).
 * @param x
 * @param y
 * @param z
 * @return quaternion.
 */
Quaternion axisAngleToQuaternion(PVector axis, float angle){
  PVector axisNormalized = new PVector(1, 0, 0);
  if(axis.mag()>0.01){
    axisNormalized = PVector.div(axis, axis.mag());
  } else { // avoid errors
    println("axis magnitude zero in 'axisAngleToQuaternion' call.");
  }
  Quaternion quaternion = new Quaternion();
  quaternion.real = cos(angle/2);
  quaternion.vector = axisNormalized.mult(sin(angle/2));
  return quaternion;
}

/**
 * Convert a rotation matrix to a quaternion representation of rotation.
 * This method takes care of some singularities, but I've noticed some rare weird behaviour that might come from this method.
 * @param matrix rotation matrix.
 * @return quaternion 
 */
Quaternion rotationMatrixToQuaternion(Matrix m){
  float angle, x, y, z;
  float epsilon = 0.01; // margin to allow for rounding errors
  float epsilon2 = 0.1; // margin to distinguish between 0 and 180 degrees
  if ((Math.abs(m.get(0, 1)-m.get(1, 0))< epsilon)
    && (Math.abs(m.get(0, 2)-m.get(2, 0))< epsilon)
    && (Math.abs(m.get(1, 2)-m.get(2, 1))< epsilon)) {
    // singularity found
    // first check for identity matrix which must have +1 for all terms
    //  in leading diagonaland zero in other terms
    if ((Math.abs(m.get(0, 1)+m.get(1, 0)) < epsilon2)
      && (Math.abs(m.get(0, 2)+m.get(2, 0)) < epsilon2)
      && (Math.abs(m.get(1, 2)+m.get(2, 1)) < epsilon2)
      && (Math.abs(m.get(0, 0)+m.get(1, 1)+m.get(2, 2)-3) < epsilon2)) {
      // this singularity is identity matrix so angle = 0
      return axisAngleToQuaternion(1, 0, 0, 0); // zero angle, arbitrary axis
    }
    // otherwise this singularity is angle = 180
    angle = PI;
    double xx = (m.get(0, 0)+1)/2;
    double yy = (m.get(1, 1)+1)/2;
    double zz = (m.get(2, 2)+1)/2;
    double xy = (m.get(0, 1)+m.get(1, 0))/4;
    double xz = (m.get(0, 2)+m.get(2, 0))/4;
    double yz = (m.get(1, 2)+m.get(2, 1))/4;
    if ((xx > yy) && (xx > zz)) { // m[0][0] is the largest diagonal term
      if (xx< epsilon) {
        x = 0;
        y = 0.7071;
        z = 0.7071;
      } else {
        x = sqrt((float)xx);
        y = (float)xy/x;
        z = (float)xz/x;
      }
    } else if (yy > zz) { // m[1][1] is the largest diagonal term
      if (yy< epsilon) {
        x = 0.7071;
        y = 0;
        z = 0.7071;
      } else {
        y = sqrt((float)yy);
        x = (float)xy/y;
        z = (float)yz/y;
      }  
    } else { // m[2][2] is the largest diagonal term so base result on this
      if (zz< epsilon) {
        x = 0.7071;
        y = 0.7071;
        z = 0;
      } else {
        z = sqrt((float)zz);
        x = (float)xz/z;
        y = (float)yz/z;
      }
    }
    return axisAngleToQuaternion(x, y, z, angle); // return 180 deg rotation
  }
  // as we have reached here there are no singularities so we can handle normally
  double s = Math.sqrt((m.get(2, 1) - m.get(1, 2))*(m.get(2, 1) - m.get(1, 2))
    +(m.get(0, 2) - m.get(2, 0))*(m.get(0, 2) - m.get(2, 0))
    +(m.get(1, 0) - m.get(0, 1))*(m.get(1, 0) - m.get(0, 1))); // used to normalise
  if (Math.abs(s) < 0.001) s=1; 
    // prevent divide by zero, should not happen if matrix is orthogonal and should be
    // caught by singularity test above, but I've left it in just in case
  angle = (float) Math.acos((m.get(0, 0) + m.get(1, 1) + m.get(2, 2) - 1)/2);
  x = (float) ((m.get(2, 1) - m.get(1, 2))/s);
  y = (float) ((m.get(0, 2) - m.get(2, 0))/s);
  z = (float) ((m.get(1, 0) - m.get(0, 1))/s);
  return axisAngleToQuaternion(x, y, z, angle);
}

/**
 * Rotate the vectorToBeRotated angle radians around the rotationAxis. 
 */
PVector rotateVector(PVector vectorToBeRotated, PVector rotationAxis, float angle){
  Quaternion rotationQuaternion = axisAngleToQuaternion(rotationAxis, angle);
  Quaternion quaternionToBeRotated = new Quaternion(0, vectorToBeRotated);
  return qMult(qConjugate(rotationQuaternion), qMult(quaternionToBeRotated, rotationQuaternion)).vector;
}

/**
 * Rotate the vectorToBeRotated using the rotationQuaternion. 
 */
PVector rotateVector(PVector vectorToBeRotated, Quaternion rotationQuaternion){
  Quaternion quaternionToBeRotated = new Quaternion(0, vectorToBeRotated);
  return qMult(qConjugate(rotationQuaternion), qMult(quaternionToBeRotated, rotationQuaternion)).vector;
}
