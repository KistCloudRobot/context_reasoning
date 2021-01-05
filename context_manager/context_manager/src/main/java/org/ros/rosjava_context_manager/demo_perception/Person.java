package org.ros.rosjava_context_manager.demo_perception;

import geometry_msgs.PoseStamped;
import geometry_msgs.Vector3;

public interface Person extends org.ros.internal.message.Message {
    static final java.lang.String _TYPE = "Person";
    static final java.lang.String _DEFINITION = "string[] objName \n string[] objConfidence \n geometry_msgs/PoseStamped objPose \n geometry_msgs/Vector3 bboxMin \n geometry_msgs/Vector3 bboxMax \n string model \n string[] affordanceName \n string[] affordanceConfidence";
/*
Later : string [] ==> String [] test
static final java.lang.String _DEFINITION = "Header header \n string[] objName \n string[] objConfidence \n geometry_msgs/PoseStamped objPose \n geometry_msgs/Vector3 bboxMin \n geometry_msgs/Vector3 bboxMax \n string model \n string[] affordanceName \n string[] affordanceConfidence";
 */

    public abstract java.util.List<String> getObjName();

    public abstract java.util.List<String> getObjConfidence();
  
    public abstract geometry_msgs.PoseStamped getObjPose();

    public abstract geometry_msgs.Vector3 getBboxMin();

    public abstract geometry_msgs.Vector3 getBboxMax();

    public abstract String getModel();

    public abstract java.util.List<String> getAffordanceName();

    public abstract java.util.List<String> getAffordanceConfidence();

}