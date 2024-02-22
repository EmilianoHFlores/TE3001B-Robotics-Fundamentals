PShape base, shoulder, upArm, loArm, end;
float rotX, rotY;
float posX=1, posY=50, posZ=50;
float alpha, beta, gamma;
float F = 50;
float T = 70;
float millisOld, gTime, gSpeed = 4;

void IK(){
  float X = posX;
  float Y = posY;
  float Z = posZ;

  float L = sqrt(Y*Y+X*X);
  float dia = sqrt(Z*Z+L*L);

  alpha = PI/2-(atan2(L, Z)+acos((T*T-F*F-dia*dia)/(-2*F*dia)));
  beta = -PI+acos((dia*dia-T*T-F*F)/(-2*F*T));
  gamma = atan2(Y, X);
}

void setTime(){
  gTime += ((float)millis()/1000 - millisOld)*(gSpeed/4);
  if(gTime >= 4)  gTime = 0;  
  millisOld = (float)millis()/1000;
}

int distance_from_arm = 65;

void writePos(int point){
  IK();
  setTime();
  TableRow row = points.getRow(point);
  float angle = 3.14 * 2 * row.getFloat("x");
  if (POLAR_COORDINATES) {
    posX = distance_from_arm * cos(angle);
    posY = distance_from_arm * sin(angle);
  }
  else {
    posX = -row.getFloat("x") * 200 + 100;
  }
  posZ = row.getFloat("y") * 200 - 100;
}

void joinEndStart(int num_points){
  float x1 = points.getRow(pointNumber-1).getFloat("x");
  float y1 = points.getRow(pointNumber-1).getFloat("y");
  float x2 = points.getRow(0).getFloat("x");
  float y2 = points.getRow(0).getFloat("y");
  // get line equation
  float m = (y2 - y1) / (x2 - x1);
  float b = y1 - m * x1;
  // create n number of points
  for (int i = 0; i < num_points; i++){
    float x = x1 + (x2 - x1) * i / num_points;
    float y = m * x + b;
    TableRow newRow = points.addRow();
    newRow.setFloat("x", x);
    newRow.setFloat("y", y);
  }
  pointNumber += num_points;
}

int MIN_OPACITY = 255;
float SPHERE_SIZE = 7;
float MIN_SPHERE_SIZE = 4;
int MAX_SIZE = 1000;
int JOIN_ENDS_SIZE = 100;
float TIP_SIZE_MULTIPLIER = 1.2;
float TIP_LENGTH_PERCENT = 0.1;
boolean POLAR_COORDINATES = false;

int POINT_JUMP = 1;
float[] Xsphere = new float[MAX_SIZE];
float[] Ysphere = new float[MAX_SIZE];
float[] Zsphere = new float[MAX_SIZE];

Table points;
int pointNumber = 0;
int end_point = 0;
int curr_point = 0;

void setup(){
    size(1200, 800, OPENGL);
    points = loadTable("points.csv", "header");
    pointNumber = points.getRowCount();
    end_point = pointNumber;

    joinEndStart(JOIN_ENDS_SIZE);

    Xsphere = new float[pointNumber];
    Ysphere = new float[pointNumber];
    Zsphere = new float[pointNumber];
    
    base = loadShape("r5.obj");
    shoulder = loadShape("r1.obj");
    upArm = loadShape("r2.obj");
    loArm = loadShape("r3.obj");
    end = loadShape("r4.obj");
    
    shoulder.disableStyle();
    upArm.disableStyle();
    loArm.disableStyle(); 
}

void draw(){ 
  if (curr_point >= pointNumber){
    curr_point = 0;
    // delete all saved points
    for (int i = 0; i < pointNumber; i++){
      Xsphere[i] = 0;
      Ysphere[i] = 0;
      Zsphere[i] = 0;
    }
  }
   writePos(curr_point);
   curr_point+=POINT_JUMP;
   background(170);
   smooth();
   lights(); 
   directionalLight(51, 102, 126, -1, 0, 0);
    
    for (int i=0; i< pointNumber - 1; i++) {
    Xsphere[i] = Xsphere[i + 1];
    Ysphere[i] = Ysphere[i + 1];
    Zsphere[i] = Zsphere[i + 1];
    }
    
    Xsphere[pointNumber - 1] = posX;
    Ysphere[pointNumber - 1] = posY;
    Zsphere[pointNumber - 1] = posZ;
   
   noStroke();
   
   translate(width/2,height/2);
   rotateX(rotX);
   rotateY(-rotY);
   scale(-4);

   int tip_points = round(pointNumber * TIP_LENGTH_PERCENT);
   int non_tip_points = pointNumber - tip_points;
   for (int i=0; i < pointNumber; i++) {
     pushMatrix();
     translate(-Ysphere[i], -Zsphere[i]-11, -Xsphere[i]);
     //Opacity Control
     int opacity = (255 * i/pointNumber);
     if (opacity < MIN_OPACITY) {
     opacity=MIN_OPACITY;
     }
     fill (#FF3B58, opacity);
     
     //Sphere Size Control
     float sphereSize;
     if (curr_point > pointNumber - JOIN_ENDS_SIZE){
      int after_end = curr_point - (pointNumber - JOIN_ENDS_SIZE);
        if(i > (pointNumber - after_end)){
          sphereSize = 0;
        }
        else{
          sphereSize = (float(pointNumber - curr_point) / float(JOIN_ENDS_SIZE)) * MIN_SPHERE_SIZE;
        }
     }
     else {
       sphereSize = MIN_SPHERE_SIZE * TIP_SIZE_MULTIPLIER * (float(i - non_tip_points) / float(tip_points));
       sphereSize = max(sphereSize, MIN_SPHERE_SIZE);
     }
     sphere (sphereSize);
     popMatrix();
    }
    
   fill(#FFE308);  
   translate(0,-40,0);   
     shape(base);
     
   translate(0, 4, 0);
   rotateY(gamma);
     shape(shoulder);
      
   translate(0, 25, 0);
   rotateY(PI);
   rotateX(alpha);
     shape(upArm);
      
   translate(0, 0, 50);
   rotateY(PI);
   rotateX(beta);
     shape(loArm);
      
   translate(0, 0, -50);
   rotateY(PI);
     shape(end);
}

void mouseDragged(){
    rotY -= (mouseX - pmouseX) * 0.01;
    rotX -= (mouseY - pmouseY) * 0.01;
}
