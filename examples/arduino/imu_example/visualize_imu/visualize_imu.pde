import processing.serial.*;
Serial myPort;

void setup()
{ 
  // Connect to a port
  String portName = "/dev/ttyUSB0";
  //String portName = "/dev/cu.usbserial-14110";
  try
  {
    myPort = new Serial(this, portName, 38400);
    print("Using port", portName);
  }
  catch(RuntimeException e1)
  {
    println("Error: Port", portName, "is not available.");
    exit();
  }
  
  // Renderer settings
  size(640, 640, P3D); 
  background(0);
  lights();
  //noStroke();
}

void draw()
{
  if ( myPort.available() > 0 )
  {
    // quaternion data: "qx,qy,qz,qw"
    String inBuffer = myPort.readString();  
    if (inBuffer != null) {
      try
      {
        background(0); // clear renderer background

        String[] q = inBuffer.split(",");
        float qx = Float.parseFloat(q[0]);
        float qy = Float.parseFloat(q[1]);
        float qz = Float.parseFloat(q[2]);
        float qw = Float.parseFloat(q[3]);
        float yaw = atan2(2*(qx*qy - qz*qw), 1-2*(qy*qy + qz*qz));
        float pitch = asin(2*(qx*qz + qy*qw));
        float roll = atan2(2*(qy*qz - qx*qw), 1-2*(qx*qx + qy*qy));
                
        //pushMatrix();
        translate(width/2, height/2, 100);
        //rotate(pitch);   // test each euler angle individually
        rotateX(roll);  // x axis right
        rotateY(pitch); // y axis up
        rotateZ(yaw);   // z axis out of screen

        box(100);
        //popMatrix();
      }
      catch(NumberFormatException e3){}
    }
  }
}
