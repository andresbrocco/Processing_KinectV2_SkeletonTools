/**
 * Method to convert X dimensions from real world to screen dimensions (meters to pixels)
 * @param meters real world dimension
 * @return screen dimension
 */
float reScaleX(float meters){
  float horizontalMargin = max(width-height, 0)/2;
  return map(meters, -2, 2, horizontalMargin-width/2, width/2-horizontalMargin);
}

/**
 * Method to convert Y dimensions from real world to screen dimensions (meters to pixels)
 * @param meters real world dimension
 * @return screen dimension
 */
float reScaleY(float meters){
  float verticalMargin = max(height-width, 0)/2;
  return map(meters, -2, 2, verticalMargin-height/2, height/2-verticalMargin);
}

/**
 * Method to convert Z dimensions from real world to screen dimensions (meters to pixels)
 * @param meters real world dimension
 * @return screen dimension
 */
float reScaleZ(float meters){
  return map(meters, 0, 4, 0, min(width, height));
}

float howCloseToTheMean(float x, float mean, float std){
  return exp(-pow((x-mean)/std, 2)/2); // Un-normalized Gaussian distribution
}

PVector calculateRelativeOrientation(Quaternion q1, Quaternion q2){ // relativeOrientation = childOrientation*inverse(parentOrientation). 
  Quaternion qRelative = qMult(q1, qConjugate(q2)); // From q1 to q2
  return quaternionToEulerAngles(qRelative);
}

PVector slerp(PVector v1, PVector v2, float step){
  float theta = PVector.angleBetween(v1, v2);
  if(sin(theta)==0){
    return v1;
  }
  float v1Multiplier = sin((1-step)*theta)/sin(theta);
  float v2Multiplier = sin(step*theta)/sin(theta);
  return PVector.add(PVector.mult(v1,v1Multiplier), PVector.mult(v2,v2Multiplier));
}

PVector pVectorAbs(PVector v){
  PVector vAbs = new PVector(0,0,0);
  vAbs.x = abs(v.x);
  vAbs.y = abs(v.y);
  vAbs.z = abs(v.z);
  return vAbs;
}

PVector pVectorExp(PVector v){
  PVector vExp = new PVector(0,0,0);
  vExp.x = exp(v.x);
  vExp.y = exp(v.y);
  vExp.z = exp(v.z);
  return vExp;
}

PVector pVectorPow(PVector v, float p){
  PVector vPow = new PVector(0,0,0);
  vPow.x = pow(v.x, p);
  vPow.y = pow(v.y, p);
  vPow.z = pow(v.z, p);
  return vPow;
}

PVector pVectorDiv(PVector v1, PVector v2){ // element-wise division
  PVector vDiv = new PVector(0,0,0);
  vDiv.x = v1.x/v2.x;
  vDiv.y = v1.y/v2.y;
  vDiv.z = v1.z/v2.z;
  return vDiv;
}

PVector pVectorMult(PVector v1, PVector v2){
  PVector vMult = new PVector(0,0,0);
  vMult.x = v1.x*v2.x;
  vMult.y = v1.y*v2.y;
  vMult.z = v1.z*v2.z;
  return vMult;
}

void drawPie3D(PVector v1, PVector v2, float size){
  float angleBetweenVectors = PVector.angleBetween(v1, v2);
  int nOfVertexes = (int)(angleBetweenVectors/(PI/12))+2;
  beginShape();
  vertex(0,0,0);
  for(int n = 0; n<nOfVertexes; n++){
    PVector auxiliarPVector = slerp(v1, v2, (float)n/ (float)(nOfVertexes-1));
    vertex(size*auxiliarPVector.x, size*auxiliarPVector.y, size*auxiliarPVector.z);
  }
  endShape(CLOSE);
}

PVector maxFromMatrix(Matrix matrix){
  double[] max = new double[3];
  double[][] matrixArray = matrix.getArray();
  for(int row=0; row < matrixArray.length; row++){
    for(int col=0; col<3; col++){
      if(matrixArray[row][col] > max[col]) max[col] = matrixArray[row][col];
    }
  }
  return new PVector((float)max[0], (float)max[1], (float)max[2]);
}

PVector minFromMatrix(Matrix matrix){
  double[] min = new double[3];
  double[][] matrixArray = matrix.getArray();
  for(int row=0; row < matrixArray.length; row++){
    for(int col=0; col<3; col++){
      if(matrixArray[row][col] < min[col]) min[col] = matrixArray[row][col];
    }
  }
  return new PVector((float)min[0], (float)min[1], (float)min[2]);
}

float maxFromArray(double[] array){
  double max = 0;
  for(int row=0; row < array.length; row++){
    if(array[row] > max) max = array[row];
  }
  return (float) max;
}

float minFromArray(double[] array){
  double min = 0;
  for(int row=0; row < array.length; row++){
    if(array[row] < min) min = array[row];
  }
  return (float) min;
}
