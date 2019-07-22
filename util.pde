/**
 * Method to convert X dimensions from real world to screen dimensions (meters to pixels)
 * @param meters real world dimension
 * @return screen dimension
 */
float reScaleX(float meters, String whoCalledMe){
  if(Float.isNaN(meters)){
    println("NaN in: "+whoCalledMe);
    exit();
  }
  float horizontalMargin = max(width-height, 0)/2;
  return map(meters, -2, 2, horizontalMargin-width/2, width/2-horizontalMargin);
}

/**
 * Method to convert Y dimensions from real world to screen dimensions (meters to pixels)
 * @param meters real world dimension
 * @return screen dimension
 */
float reScaleY(float meters, String whoCalledMe){
  if(Float.isNaN(meters)){
    println("NaN in: "+whoCalledMe);
    exit();
  }
  float verticalMargin = max(height-width, 0)/2;
  return map(meters, -2, 2, verticalMargin-height/2, height/2-verticalMargin);
}

/**
 * Method to convert Z dimensions from real world to screen dimensions (meters to pixels)
 * @param meters real world dimension
 * @return screen dimension
 */
float reScaleZ(float meters, String whoCalledMe){
  if(Float.isNaN(meters)){
    println("NaN in: "+whoCalledMe);
    exit();
  }
  return map(meters, 0, 4, 0, min(width, height));
}

/**
 * Given a distribution (mean and standard deviation), find how close a sample x is to the mean.
 * @param x sample to be evaluated.
 * @param mean distribution average.
 * @param std distribution standard deviation.
 * @return 1 if x=mean; 0 if x=far from mean;
 */
float howCloseToTheMean(float x, float mean, float std){
  return exp(-pow((x-mean)/std, 2)/2); // Un-normalized Gaussian distribution
}

/**
 * Calculate Euler angles between orientation q1 and orientation q2.
 * @param q1 first orientation quaternion.
 * @param q2 second orientation quaternion.
 * @return PVector of Euler Angles from q1 to q2.
 */
PVector calculateRelativeOrientation(Quaternion q1, Quaternion q2){ // relativeOrientation = childOrientation*inverse(parentOrientation). 
  Quaternion qRelative = qMult(q1, qConjugate(q2)); // From q1 to q2
  return quaternionToEulerAngles(qRelative);
}

/**
 * Spherical linear interpolation/extrapolation for PVectors.
 * @param v1 PVector 1.
 * @param v2 PVector 2.
 * @param step percentage of path from v1 to v2.
 * @return PVector between v1 and v2 (if 0<step<1).
 */
PVector slerp(PVector v1, PVector v2, float step){
  float theta = PVector.angleBetween(v1, v2);
  if(sin(theta)==0){
    return v1;
  }
  float v1Multiplier = sin((1-step)*theta)/sin(theta);
  float v2Multiplier = sin(step*theta)/sin(theta);
  return PVector.add(PVector.mult(v1,v1Multiplier), PVector.mult(v2,v2Multiplier));
}

/**
 * Draw a flat 3-dimensional "pie-chart" around the current coordinate system. 
 * @param v1 direction of begining of the pie.
 * @param v2 direction of end of the pie.
 * @param size radius of the pie.
 */
void drawPie3D(PVector v1, PVector v2, float size){
  float angleBetweenVectors = PVector.angleBetween(v1, v2);
  int nOfVertexes = (int)(angleBetweenVectors/(PI/12))+2;
  beginShape();
  vertex(0,0,0);
  for(int n = 0; n<nOfVertexes; n++){
    PVector auxiliarPVector = slerp(v1, v2, (float)n/ (float)(nOfVertexes-1));
    vertex(reScaleX(size*auxiliarPVector.x, "drawPie3D"), reScaleY(size*auxiliarPVector.y, "drawPie3D"), reScaleZ(size*auxiliarPVector.z, "drawPie3D"));
  }
  endShape(CLOSE);
}

/**
 * Auxiliar function to call the vertex function with a PVector.
 * @param vertex PVector.
 * @param whoCalledMe string for debbugging.
 */
void vertex(PVector vertex, String whoCalledMe){
  vertex(reScaleX(vertex.x, whoCalledMe), reScaleY(vertex.y, whoCalledMe), reScaleZ(vertex.z, whoCalledMe));
}

/**
 * Return the signal of a float (-1 or 1).
 * @param x value to get the signal from.
 * @return sign(x) the signal of x.
 */
int sign(float x){
  if(x<0) return -1;
  else return 1;
}
