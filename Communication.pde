import oscP5.*;
import netP5.*;
import java.util.Iterator;

int pdPort = 3000;
int myPort = 3001;

Communication communication = new Communication("127.0.0.1", pdPort, myPort);
  
public class Communication{
  private OscP5 oscP5;
  private NetAddress pdAddress;
  
  public Communication(String pdIp, int pdPort, int myPort){
    this.oscP5 = new OscP5(this, myPort);
    this.pdAddress = new NetAddress(pdIp, pdPort); //localhost: "127.0.0.1" "192.168.15.16" // "192.168.15.1"
  }
  
  public void sendScene(Scene scene){
    if(!scene.activeSkeletons.isEmpty()){
      Iterator<HashMap.Entry<Integer, Skeleton>> activeSkeletonsIterator = scene.activeSkeletons.entrySet().iterator();
      while (activeSkeletonsIterator.hasNext()) {
        HashMap.Entry<Integer, Skeleton> entry = activeSkeletonsIterator.next();
        this.sendKinectSkeleton(entry.getValue());
      }
    }
  }
  
  private void sendKinectSkeleton(Skeleton skeleton){ 
    OscMessage messageToPd = new OscMessage("/indexColor:");
    messageToPd.add(skeleton.indexColor);
    this.oscP5.send(messageToPd, pdAddress);
    messageToPd = new OscMessage("/isTracked:");
    messageToPd.add(skeleton.isTracked);
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
      messageToPd.add(skeleton.joints[jointType].currentEstimatedOrientation.real);
      messageToPd.add(skeleton.joints[jointType].currentEstimatedOrientation.vector.x);
      messageToPd.add(skeleton.joints[jointType].currentEstimatedOrientation.vector.y);
      messageToPd.add(skeleton.joints[jointType].currentEstimatedOrientation.vector.z);
      this.oscP5.send(messageToPd, pdAddress);
    }
  }
}
