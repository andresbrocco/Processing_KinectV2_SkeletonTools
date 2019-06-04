import oscP5.*;
import netP5.*;
  
public class Communication{
  private OscP5 oscP5;
  private NetAddress pdAddress;
  
  
  public Communication(String pdIp, int pdPort, int myPort){
    this.oscP5 = new OscP5(this, myPort);
    this.pdAddress = new NetAddress(pdIp, pdPort); //localhost: "127.0.0.1" "192.168.15.16" // "192.168.15.1"
  }
  
  public void sendScene(Scene scene){
    if(!scene.activeSkeletons.isEmpty()){
      for(Skeleton skeleton:scene.activeSkeletons.values()){
        this.sendKinectSkeleton(skeleton);
        this.sendGrainParameters(skeleton);
        this.sendVideoParameter(skeleton);
        this.sendSteeringWheel(skeleton);
      }
    }
  }
  
  private void sendSteeringWheel(Skeleton skeleton){
    OscMessage messageToVideoSphere = new OscMessage("/steeringWheelRollStep:");
    messageToVideoSphere.add(skeleton.features.steeringWheel.rollStep);
    this.oscP5.send(messageToVideoSphere, pdAddress);
    messageToVideoSphere = new OscMessage("/steeringWheelPitchStep:");
    messageToVideoSphere.add(skeleton.features.steeringWheel.pitchStep);
    this.oscP5.send(messageToVideoSphere, pdAddress);
  }
  
  private void sendKinectSkeleton(Skeleton skeleton){ 
    OscMessage messageToPd = new OscMessage("/indexColor:");
    messageToPd.add(skeleton.indexColor);
    this.oscP5.send(messageToPd, pdAddress);
    
    messageToPd = new OscMessage("/handStates:");
    messageToPd.add(skeleton.estimatedHandRadius[0]);
    messageToPd.add(skeleton.estimatedHandRadius[1]);
    this.oscP5.send(messageToPd, pdAddress);
    
    for (int jointType = 0; jointType<25 ; jointType++){
      messageToPd = new OscMessage("/joint" + Integer.toString(jointType) + ":");
      messageToPd.add(skeleton.joints[jointType].trackingState);
      messageToPd.add(skeleton.joints[jointType].estimatedPosition.x);
      messageToPd.add(skeleton.joints[jointType].estimatedPosition.y);
      messageToPd.add(skeleton.joints[jointType].estimatedPosition.z);
      messageToPd.add(skeleton.joints[jointType].estimatedOrientation.real);
      messageToPd.add(skeleton.joints[jointType].estimatedOrientation.vector.x);
      messageToPd.add(skeleton.joints[jointType].estimatedOrientation.vector.y);
      messageToPd.add(skeleton.joints[jointType].estimatedOrientation.vector.z);
      this.oscP5.send(messageToPd, pdAddress);
    }
  }
  
  private void sendGrainParameters(Skeleton skeleton){
    OscMessage messageToPd = new OscMessage("/Ready");
    messageToPd = new OscMessage("/mid_z");
    messageToPd.add(map((skeleton.joints[SPINE_BASE].estimatedPosition.z),0.4,3.5,0,1));
    this.oscP5.send(messageToPd, pdAddress);
    messageToPd = new OscMessage("/hand_left_x");
    messageToPd.add(map((skeleton.joints[HAND_LEFT].estimatedPosition.x),-1.5,1,0,1));
    this.oscP5.send(messageToPd, pdAddress);
    messageToPd = new OscMessage("/hand_left_y");
    messageToPd.add(map((skeleton.joints[HAND_LEFT].estimatedPosition.y),-1.5,1,0,1));
    this.oscP5.send(messageToPd, pdAddress);
    messageToPd = new OscMessage("/hand_right_x");
    messageToPd.add(map((skeleton.joints[HAND_RIGHT].estimatedPosition.x),-1.5,1,0,1));
    this.oscP5.send(messageToPd, pdAddress);
    messageToPd = new OscMessage("/hand_right_y");
    messageToPd.add(map((skeleton.joints[HAND_RIGHT].estimatedPosition.y),-1.5,1,0,1));
    this.oscP5.send(messageToPd, pdAddress);
  }
  
  private void sendVideoParameter(Skeleton skeleton){
    OscMessage messageToPd = new OscMessage("/Ready");  
    this.oscP5.send(messageToPd, pdAddress);
    messageToPd = new OscMessage("/Elastic");
    messageToPd.add((skeleton.features.distanceBetweenHands));
    this.oscP5.send(messageToPd,pdAddress);
  }
}
