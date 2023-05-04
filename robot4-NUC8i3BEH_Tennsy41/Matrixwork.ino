void matrix_work()
{
  Update_Matrix[0][0] = 0;
  Update_Matrix[0][1] = -dt * gzr; //-z
  Update_Matrix[0][2] = dt * gyr; //y
  Update_Matrix[1][0] = dt * gzr; //z
  Update_Matrix[1][1] = 0;
  Update_Matrix[1][2] = -dt * gxr;
  Update_Matrix[2][0] = -dt * gyr;
  Update_Matrix[2][1] = dt * gxr;
  Update_Matrix[2][2] = 0;
  Matrix_Multiply(DCM_Matrix, Update_Matrix, Temporary_Matrix); //a*b=c
  for (int x = 0; x < 3; x++) //Matrix Addition (update)
  {
    for (int y = 0; y < 3; y++)
    {
      DCM_Matrix[x][y] += Temporary_Matrix[x][y];
    }
  }
}


void Normalize(void)
{
  float error = 0;
  float temporary[3][3];
  float renorm = 0;
  error = -Vector_Dot_Product(&DCM_Matrix[0][0], &DCM_Matrix[1][0]) * .5; //eq.19
  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
  Vector_Cross_Product(&temporary[2][0], &temporary[0][0], &temporary[1][0]); // c= a x b //eq.20
  renorm = .5 * (3 - Vector_Dot_Product(&temporary[0][0], &temporary[0][0])); //eq.21
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  renorm = .5 * (3 - Vector_Dot_Product(&temporary[1][0], &temporary[1][0])); //eq.21
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  renorm = .5 * (3 - Vector_Dot_Product(&temporary[2][0], &temporary[2][0])); //eq.21
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

// Computes the dot product of two vectors
float Vector_Dot_Product(const float v1[3], const float v2[3])
{
  float result = 0;
  for (int c = 0; c < 3; c++)
  {
    result += v1[c] * v2[c];
  }
  return result;
}

// Computes the cross product of two vectors
// out has to different from v1 and v2 (no in-place)!
void Vector_Cross_Product(float out[3], const float v1[3], const float v2[3])
{
  out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

// Multiply the vector by a scalar
void Vector_Scale(float out[3], const float v[3], float scale)
{
  for (int c = 0; c < 3; c++)
  {
    out[c] = v[c] * scale;
  }
}

// Adds two vectors
void Vector_Add(float out[3], const float v1[3], const float v2[3])
{
  for (int c = 0; c < 3; c++)
  {
    out[c] = v1[c] + v2[c];
  }
}


void Matrix_Multiply(const float a[3][3], const float b[3][3], float out[3][3])
{
  for (int x = 0; x < 3; x++) // rows
  {
    for (int y = 0; y < 3; y++) // columns
    {
      out[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y];
    }
  }
}

void gyroread()
{
  t2 = micros() - t1;
  t1 = micros();
  dt = float(t2) / 1000000;
  gyro.readSensor();
  gxr = gyro.getGyroX_rads();
  gyr = gyro.getGyroY_rads();
  gzr = gyro.getGyroZ_rads();
  temp1 = gxr * 100;
  gxr = float(temp1) / 100;
  temp1 = gyr * 100;
  gyr = float(temp1) / 100;
  temp1 = gzr * 100;
  gzr = float(temp1) / 100;

  matrix_work();
  Normalize();
  yaw2 = atan2(DCM_Matrix[1][0], DCM_Matrix[0][0]);
}
