/*
 * Copyright (C) 2014 ailab.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.rosjava_context_manager;

//import javax.management.monitor.Monitor;
import rosjava_custom_srv.*;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.jpl7.*;
import java.util.*;
import java.io.PrintStream;

/**
 * A simple {@link Subscriber} {@link NodeMain}.
 */
public class ContextListenerGazebo extends AbstractNodeMain {

	int visualObjectPerceptionCount = 0;
	int visualRobotBodyPerceptionCount = 0;
	int visualRobotHandPerceptionCount = 0;
	int jointPerceptionCount = 0;

	int visualObjectPerceptionInterval=0;
	int visualRobotBodyPerceptionInterval=0;
	int visualRobotHandPerceptionInterval=0;
	int jointPerceptionInterval=0;

	int removeTime=10;
	int removeInterval=5;

	static List<String> oSubject = new ArrayList<String>();
	static List<String>  oProperty = new ArrayList<String>();
	static List<String>  oObject = new ArrayList<String>();
	static int[] oStatus = new int[1000];
	static int oInd;

	static List<String>  bSubject = new ArrayList<String>();
	static List<String>  bProperty = new ArrayList<String>();
	static List<String>  bObject = new ArrayList<String>();
	static int[] bStatus = new int[1000]; 
	static int bInd;

	static List<String>  hSubject = new ArrayList<String>();
	static List<String>  hProperty = new ArrayList<String>();
	static List<String>  hObject = new ArrayList<String>();
	static int[] hStatus = new int[1000];
	static int hInd;

	static List<String>  jSubject = new ArrayList<String>();
	static List<String>  jProperty = new ArrayList<String>();
	static List<String>  jObject = new ArrayList<String>();
	static int[] jStatus = new int[1000];
	static int jInd;

	static List<String>  fsdSubject = new ArrayList<String>();
	static List<String>  fsdProperty = new ArrayList<String>();
	static List<String>  fsdObject = new ArrayList<String>();
	static int[] fsdStatus = new int[1000];
	static int fsdInd;

  static int[][] oIdCount = new int[50][2];
	static int oIdInd;

	static int[][] bIdCount = new int[50][2];
	static int bIdInd;

	static int[][] hIdCount = new int[50][2];
	static int hIdInd;

	final static int queueSize=1000;

	Scanner scan;

	static Publisher<QueryServiceRequest> questioner;
	static Publisher<MainServiceRequest> requestor;
	static Subscriber<MonitorServiceResponse> receiver;
	static Subscriber<QueryServiceResponse> answerer;
	static Log log;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("context_listener");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		scan = new Scanner(System.in);
		log = connectedNode.getLog();

		Subscriber<std_msgs.Float32MultiArray> object_subscriber = connectedNode.newSubscriber("/camera_info",
				std_msgs.Float32MultiArray._TYPE);// Subscriber node --> object

        Subscriber<std_msgs.Float32MultiArray> robot_body_subscriber = connectedNode.newSubscriber("/robot_info/body",
				std_msgs.Float32MultiArray._TYPE);// Subscriber node --> jackal base

        Subscriber<std_msgs.Float32MultiArray> robot_hand_subscriber = connectedNode.newSubscriber("/robot_info/hand",
				std_msgs.Float32MultiArray._TYPE);// Subscriber node --> jaco hand

		Subscriber<sensor_msgs.JointState> robot_joint_subscriber = connectedNode
				.newSubscriber("/joint_info", sensor_msgs.JointState._TYPE);
		//receiver = connectedNode.newSubscriber("ContextManager/Monitor/ProvisionForPM", MonitorServiceResponse._TYPE);
		answerer = connectedNode.newSubscriber("context_manager/query/provision_for_pm", QueryServiceResponse._TYPE);

		questioner = connectedNode.newPublisher("context_manager/query/reception", QueryServiceRequest._TYPE);
		requestor = connectedNode.newPublisher("context_manager/main/reception", MainServiceRequest._TYPE);

	

/*
		questioner.setQueueLimit(queueSize);
		requestor.setQueueLimit(queueSize);
		*/

		/*
		 * System.out.println(t + " " + (Query.hasSolution(t) ? "succeeded" :
		 * "failed")); System.out.println("SWI-Prolog working from now on!!!");
		 */

		// VisualObjectPerception

		answerer.addMessageListener(new MessageListener<QueryServiceResponse>() {
			@Override
			public void onNewMessage(QueryServiceResponse response) {
				String ans = response.getResult();
				log.info(String.format("The response is : " + ans));
			}
		},queueSize);

		/*receiver.addMessageListener(new MessageListener<MonitorServiceResponse>() {
			@Override
			public void onNewMessage(MonitorServiceResponse response) {
				String res = response.getPredicate() + " " + response.getParam1() + " " + response.getParam2() + " "
						+ response.getParam3() + " " + response.getParam4();
				res += " " + response.getStatus();
				log.info(String.format("The response is : " + res));

			}
		});
		*/
		
		object_subscriber.addMessageListener(new MessageListener<std_msgs.Float32MultiArray>() {
			@Override
			public void onNewMessage(std_msgs.Float32MultiArray message) {
				//String[] subject = new String [50];
				//String[] property = new String [50];
				//String[] object = new String[50];
			for(int k=0;k<message.getData().length;k+=8){
				int subVisualObjectPerceptionCount=0;
				String result = "";
				String assertString = "";
				float x = message.getData()[0+k];
				float y = message.getData()[1+k];
				float z = message.getData()[2+k];
				float quart_x = message.getData()[3+k];
				float quart_y = message.getData()[4+k];
				float quart_z = message.getData()[5+k];
				float quart_w = message.getData()[6+k];//THIS ISN'T used ___JS
				float IDN = message.getData()[7+k];
				String ID = "";
				String type = "";
				int time = 0;

				// radian to degree
				float a = (float) Math.toDegrees(quart_x);
				float b = (float) Math.toDegrees(quart_y);
				float c = (float) Math.toDegrees(quart_z);


				/*if (IDN == (float) 100) {
				//	ID = "coffee_can_1";
				//  oIdInd = 0;
				//} else if (IDN == (float) 101) {
				//	ID = "coffee_can_2";
				//  oIdInd = 1;
				//} else if (IDN == (float) 102) {
				//	ID = "coffee_can_3";
				//  oIdInd = 2;
				//} else if (IDN == (float) 103) {
				//	ID = "potted_meat_can_1";
				//  oIdInd = 3;
				//} else if (IDN == (float) 104) {
				//	ID = "potted_meat_can_2";
				//  oIdInd = 4;
				//} 
				// else if (IDN == (float) 105) {
				// 	ID = "potted_meat_can_3";
				//  oIdInd = 5;
				// } else if (IDN == (float) 106) {
				// 	ID = "tea_box_1";
				//  oIdInd = 6;
				// } else if (IDN == (float) 107) {
				// 	ID = "tea_box_2";
				//  oIdInd = 7;
				 } else */if (IDN == (float) 108) {
				 	ID = "water_bottle_1";
					oIdInd = 8;
				 } else if (IDN == (float) 109) {
				 	ID = "tomato_soup_can_2";
					oIdInd = 9;
				 } else if (IDN == (float) 110) {
				 	ID = "tomato_soup_can_1";
					oIdInd = 10;
				 //} else if (IDN == (float) 111) {
				 //	ID = "cracker_box_1" ;
				//	oIdInd = 11;
				// } else if (IDN == (float) 112) {
				// 	ID = "cracker_box_2";
				//	oIdInd = 12;
				 } else if (IDN == (float) 113) {
				 	ID = "water_bottle_2";
					oIdInd = 13;
				 } else if (IDN == (float) 114) {
				 	ID = "water_bottle_3";
					oIdInd = 14;
				 } else if (IDN == (float) 115) {
				 	ID = "cracker_box_3";
					oIdInd = 15;
				 } 
				else {
				//	ID="nothing";
				 	oIdInd = 49;
				}

				//visualObjectPerceptionCount++;//=oIdCount[oIdInd][1]++;
				subVisualObjectPerceptionCount=oIdCount[oIdInd][1]++;
				visualObjectPerceptionCount=subVisualObjectPerceptionCount;

				time = (int) (System.currentTimeMillis() / 1000);//currentTimeMillis() : 1/1000 s return --> 0m 1s = 100 --> 1s  

				//result = "[Perception_Manager] " + "time:"+time+"Object_name:" + ID + ", Object Pose(X: " + x + " Y: "+ y+ " Z: " + z + "), Object Orientation(A: " + quart_x+ " B: " + quart_y+ " G: " + quart_z+ ")";
//result = "[Perception_Manager] " + "time:"+time+"Object_name:" + ID + ", Object Pose(X: " + x + " Y: "+ y+ " Z: " + z + "), Object Orientation(A: " + quart_x+ " B: " + quart_y+ " G: " + quart_z+ ")";

			/*
				subject[ind]="'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception"+ (++visualObjectPerceptionCount);
				property[ind]="'http://www.w3.org/1999/02/22-rdf-syntax-ns#type'";
				object[ind]="'http://knowrob.org/kb/knowrob.owl#VisualObjectPerception'";
				status[ind++]=3;
				
				// startTime
				subject[ind]="'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception"	+ (visualObjectPerceptionCount);
				property[ind]="'http://knowrob.org/kb/knowrob.owl#startTime'";
				object[ind]="'http://www.arbi.com/ontologies/arbi.owl#timepoint_"+ time + "'";
				status[ind++]=3;

				subject[ind]="'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception";
				property[ind]="'http://knowrob.org/kb/knowrob.owl#startTime'";
				object[ind]="'http://www.arbi.com/ontologies/arbi.owl#timepoint_"+ time + "'";
				status[ind++]=5;

				// objectActedOn
				subject[ind]="'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception"+ visualObjectPerceptionCount;
				property[ind]="'http://knowrob.org/kb/knowrob.owl#objectActedOn'";
				object[ind]="'http://www.arbi.com/ontologies/arbi.owl#"+ ID + "'";
				status[ind++]=3;

				// eventOccursAt
				subject[ind]="'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception"+ visualObjectPerceptionCount;
				property[ind]="'http://knowrob.org/kb/knowrob.owl#objectActedOn'";
				object[ind]="'http://www.arbi.com/ontologies/arbi.owl#"+ ID + "'";
				status[ind++]=3;
				*/
			
				
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception" +"_"+ID+"_"
						+ visualObjectPerceptionCount
						+ "' 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' 'http://knowrob.org/kb/knowrob.owl#VisualObjectPerception' o";
				assertTriple(assertString);
				// startTime
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception"+"_"+ID+"_"
						+ visualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#startTime' 'http://www.arbi.com/ontologies/arbi.owl#timepoint_"
						+ time + "' o";
				assertTriple(assertString);

				assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception"+"_"+ID
						+ "' 'http://knowrob.org/kb/knowrob.owl#startTime' 'http://www.arbi.com/ontologies/arbi.owl#timepoint_"
						+ time + "' o";
				updateCurrentTriple(assertString);
				// objectActedOn
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception"+"_"+ID+"_"
						+ visualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#objectActedOn' 'http://www.arbi.com/ontologies/arbi.owl#"
						+ ID + "' o";
				assertTriple(assertString);
				// eventOccursAt
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception"+"_"+ID+"_"
						+ visualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#eventOccursAt' 'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_"
						+ ID + subVisualObjectPerceptionCount + "' o";
				assertTriple(assertString);

				assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception"+"_"+ID
						+ "' 'http://knowrob.org/kb/knowrob.owl#eventOccursAt' 'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_"
						+ ID + "' o";
				updateCurrentTriple(assertString);
				// rotationMatrix3D
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ subVisualObjectPerceptionCount
						+ "' 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' 'http://knowrob.org/kb/knowrob.owl#RotationMatrix3D' o";
				assertTriple(assertString);
				// x,y,z
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ subVisualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#m03' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ x + "')) o";
				assertTriple(assertString);
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ "' 'http://knowrob.org/kb/knowrob.owl#m03' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ x + "')) o";
				updateCurrentTriple(assertString);
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ subVisualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#m13' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ y + "')) o";
				assertTriple(assertString);
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ "' 'http://knowrob.org/kb/knowrob.owl#m13' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ y + "')) o";
				updateCurrentTriple(assertString);
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ subVisualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#m23' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ z + "')) o";
				assertTriple(assertString);
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ "' 'http://knowrob.org/kb/knowrob.owl#m23' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ z + "')) o";
				updateCurrentTriple(assertString);
				// a,b,g
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ subVisualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#m02' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ a + "')) o";
				assertTriple(assertString);
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ "' 'http://knowrob.org/kb/knowrob.owl#m02' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ a + "')) o";
				updateCurrentTriple(assertString);
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ subVisualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#m12' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ b + "')) o";
				assertTriple(assertString);
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ "' 'http://knowrob.org/kb/knowrob.owl#m12' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ b + "')) o";
				updateCurrentTriple(assertString);
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ subVisualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#m22' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ c + "')) o";
				assertTriple(assertString);
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ "' 'http://knowrob.org/kb/knowrob.owl#m22' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ c + "')) o";
				updateCurrentTriple(assertString);
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ subVisualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#m32' literal(type('http://www.w3.org/2001/XMLSchema#double','0')) o";
				assertTriple(assertString);
				// print data
				
				result = "[Perception_Manager/visual_object_perception] time:"+time+
				"  Object_name:" + ID + ", Object Pose(X: " + x + " Y: "+ y+ " Z: " + z + "),Object Orientation(A: " + 
				a+ " B: " + b+ " G: " + c;
				 //System.out.println(result);
				


				if(visualObjectPerceptionCount%removeTime==0){
					System.out.println(visualObjectPerceptionCount);
						for(int i=visualObjectPerceptionInterval;i<visualObjectPerceptionCount-removeInterval;i++){
							retractTriple("'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception"+"_"+ID+"_"+i+"' A"+" B o");
							//retractTriple("'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID+i+"' A"+" B o");
						}
						if(visualObjectPerceptionInterval<visualObjectPerceptionCount-removeInterval)
							visualObjectPerceptionInterval=visualObjectPerceptionCount-removeInterval;
					//	sleep(5);
					}
					if(oIdCount[oIdInd][1]%removeTime==0){
						for(int i=oIdCount[oIdInd][0];i<oIdCount[oIdInd][1]-removeInterval;i++){
							
							retractTriple("'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID+i+"' A"+" B o");
						}
						oIdCount[oIdInd][0]=oIdCount[oIdInd][1]-removeInterval;
					
					}
			}

					sendMessage("ob");
			}
		},queueSize);

		// JointPerception
		robot_joint_subscriber.addMessageListener(new MessageListener<sensor_msgs.JointState>() {

			@Override
			public void onNewMessage(sensor_msgs.JointState message) {
				int time = message.getHeader().getStamp().secs;
				String assertString = "";
				String name;
				double position;
				double velocity;
				double effort;
				String result_joint;

				//test
				//if((jointPerceptionCount++)%10==0)
				for (int i = 0; i < message.getName().size(); i++) {// 크기만큼 반복
					name = message.getName().get(i);
					position = message.getPosition()[i];
					position = Math.toDegrees(position);
					if (message.getVelocity().length > 0)
						velocity = message.getVelocity()[i];
					else
						velocity = 0;
					if (message.getEffort().length > 0)
						effort = message.getEffort()[i];
					else
						effort = 0;

					// assert data
					// type
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#jointPerception" + (++jointPerceptionCount)
							+ "' 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' 'http://knowrob.org/kb/knowrob.owl#JointPerception' j";
					assertTriple(assertString);
					// startTime
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#jointPerception" + jointPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#startTime' 'http://www.arbi.com/ontologies/arbi.owl#timepoint_"
							+ time + "' j";
					assertTriple(assertString);
					/*assertString = "'http://www.arbi.com/ontologies/arbi.owl#jointPerception"
							+ "' 'http://knowrob.org/kb/knowrob.owl#startTime' 'http://www.arbi.com/ontologies/arbi.owl#timepoint_"
							+ time + "' j";
					updateCurrentTriple(assertString);*/
					// objectActedOn
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#jointPerception" + jointPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#objectActedOn' 'http://www.arbi.com/ontologies/arbi.owl#"
							+ name + "' j";
					assertTriple(assertString);
					// radius
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#jointPerception" + jointPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#radius' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ position + "')) j";
					assertTriple(assertString);
					/*assertString = "'http://www.arbi.com/ontologies/arbi.owl#jointPerception"
							+ "' 'http://knowrob.org/kb/knowrob.owl#radius' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ position + "')) j";
					updateCurrentTriple(assertString);*/
					// velocity
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#jointPerception" + jointPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#velocity' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ velocity + "')) j";
					assertTriple(assertString);
					/*assertString = "'http://www.arbi.com/ontologies/arbi.owl#jointPerception"
							+ "' 'http://knowrob.org/kb/knowrob.owl#velocity' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ velocity + "')) j";
					updateCurrentTriple(assertString);*/
					// effort
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#jointPerception" + jointPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#effort' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ effort + "')) j";
					assertTriple(assertString);
					/*assertString = "'http://www.arbi.com/ontologies/arbi.owl#jointPerception"
							+ "' 'http://knowrob.org/kb/knowrob.owl#effort' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ effort + "')) j";
					updateCurrentTriple(assertString);*/

					
					// print data
					 result_joint = "[Perception_Manager/jointPerception] " + "time:"+time+", Joint Name:" + name + ", Joint radius:("+position+"), Joint Velocity("+velocity+"), Joint Effort:("+effort+")";
					 System.out.println(result_joint);
					// sleep(15);
				

				if(jointPerceptionCount%(2*message.getName().size())==0){
						for(int k=jointPerceptionInterval;k<jointPerceptionCount-message.getName().size();k++)
							retractTriple("'http://www.arbi.com/ontologies/arbi.owl#jointPerception"+k+"' A"+" B j");
						jointPerceptionInterval=jointPerceptionCount-message.getName().size();
					//	sleep(5);
					}
					sendMessage("jb");
			}}
		},queueSize);
        
		robot_body_subscriber.addMessageListener(new MessageListener<std_msgs.Float32MultiArray>() {
			@Override
			public void onNewMessage(std_msgs.Float32MultiArray message) {
				int subVisualRobotBodyPerceptionCount=0;
				String result = "";
				String assertString = "";
				float x = message.getData()[0];
				float y = message.getData()[1];
				float z = message.getData()[2];
				float quart_x = message.getData()[3];
				float quart_y = message.getData()[4];
				float quart_z = message.getData()[5];
				float quart_w = message.getData()[6];
				String ID = "";
				String type = "";
				int time = 0;

				// radian to degree
				float a = (float) Math.toDegrees(quart_x);
				float b = (float) Math.toDegrees(quart_y);
				float c = (float) Math.toDegrees(quart_z);


				if (message.getData()[7] == (float) 1) {
					ID = "jackal_1";
					bIdInd=0;
				} else {
					bIdInd=49;
				}

				visualRobotBodyPerceptionCount++;
				subVisualRobotBodyPerceptionCount=bIdCount[bIdInd][1]++;

				time = (int) (System.currentTimeMillis() / 1000);

					assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotBodyPerception"
							+ visualRobotBodyPerceptionCount
							+ "' 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' 'http://knowrob.org/kb/knowrob.owl#VisualRobotBodyPerception' b";
					assertTriple(assertString);
					// startTime
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotBodyPerception"
							+ visualRobotBodyPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#startTime' 'http://www.arbi.com/ontologies/arbi.owl#timepoint_"
							+ time + "' b";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotBodyPerception"
							+ "' 'http://knowrob.org/kb/knowrob.owl#startTime' 'http://www.arbi.com/ontologies/arbi.owl#timepoint_"
							+ time + "' b";
					updateCurrentTriple(assertString);
					// objectActedOn
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotBodyPerception"
							+ visualRobotBodyPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#objectActedOn' 'http://www.arbi.com/ontologies/arbi.owl#"
							+ ID + "' b";
					assertTriple(assertString);
					// eventOccursAt
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotBodyPerception"
							+ visualRobotBodyPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#eventOccursAt' 'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_"
							+ ID + subVisualRobotBodyPerceptionCount + "' b";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotBodyPerception"
							+ "' 'http://knowrob.org/kb/knowrob.owl#eventOccursAt' 'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_"
							+ ID + "' b";
					updateCurrentTriple(assertString);
					// rotationMatrix3D
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotBodyPerceptionCount
							+ "' 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' 'http://knowrob.org/kb/knowrob.owl#RotationMatrix3D' b";
					assertTriple(assertString);
					// x,y,z
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotBodyPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m03' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ x + "')) b";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ "' 'http://knowrob.org/kb/knowrob.owl#m03' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ x + "')) b";
					updateCurrentTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotBodyPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m13' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ y + "')) b";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ "' 'http://knowrob.org/kb/knowrob.owl#m13' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ y + "')) b";
					updateCurrentTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotBodyPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m23' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ z + "')) b";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ "' 'http://knowrob.org/kb/knowrob.owl#m23' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ z + "')) b";
					updateCurrentTriple(assertString);
					// a,b,g
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotBodyPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m02' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ a + "')) b";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ "' 'http://knowrob.org/kb/knowrob.owl#m02' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ a + "')) b";
					updateCurrentTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotBodyPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m12' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ b + "')) b";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ "' 'http://knowrob.org/kb/knowrob.owl#m12' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ b + "')) b";
					updateCurrentTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotBodyPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m22' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ c + "')) b";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ "' 'http://knowrob.org/kb/knowrob.owl#m22' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ c + "')) b";
					updateCurrentTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotBodyPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m32' literal(type('http://www.w3.org/2001/XMLSchema#double','0')) b";
					assertTriple(assertString);

					//sleep(20);

					if(visualRobotBodyPerceptionCount%20==0){
						for(int i=visualRobotBodyPerceptionInterval;i<visualRobotBodyPerceptionCount-10;i++){
							retractTriple("'http://www.arbi.com/ontologies/arbi.owl#visualRobotBodyPerception"+i+"' A"+" B b");
							//retractTriple("'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID+i+"' A"+" B");
						}
						if(visualRobotBodyPerceptionInterval<visualRobotBodyPerceptionCount-10)
							visualRobotBodyPerceptionInterval=visualRobotBodyPerceptionCount-10;
					
					//	sleep(5);
					}

					if(bIdCount[bIdInd][1]%removeTime==0){
						for(int i=bIdCount[bIdInd][0];i<bIdCount[bIdInd][1]-removeInterval;i++){
							
							retractTriple("'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID+i+"' A"+" B b");
						}
						bIdCount[bIdInd][0]=bIdCount[bIdInd][1]-removeInterval;
					//	sleep(5);
					}

					sendMessage("bb");
				result = "[Perception_Manager/robot_body_perception] time:"+time+
				"  Object_name:" + ID + ", Object Pose(X: " + x + " Y: "+ y+ " Z: " + z + "),Object Orientation(A: " + 
				a+ " B: " + b+ " G: " + c;
				// System.out.print(result);
			}
		},queueSize);

        robot_hand_subscriber.addMessageListener(new MessageListener<std_msgs.Float32MultiArray>() {
			@Override
			public void onNewMessage(std_msgs.Float32MultiArray message) {
				int subVisualRobotHandPerceptionCount=0;
				String result = "";
				String assertString = "";
				float x = message.getData()[0];
				float y = message.getData()[1];
				float z = message.getData()[2];
				float quart_x = message.getData()[3];
				float quart_y = message.getData()[4];
				float quart_z = message.getData()[5];
				float quart_w = message.getData()[6];
				float finger_x = message.getData()[7];
				float finger_y = message.getData()[8];
				float finger_z = message.getData()[9];
				String ID = "";
				String spaceName="space_surrounded_by_hand";
				String type = "";
				int time = 0;

				// radian to degree
				float a = (float) Math.toDegrees(quart_x);
				float b = (float) Math.toDegrees(quart_y);
				float c = (float) Math.toDegrees(quart_z);


				if (message.getData()[10] == (float) 0) {
					ID = "jaco_1";
					hIdInd=0;
				} else {
					hIdInd=49;
				}

				visualRobotHandPerceptionCount++;
				subVisualRobotHandPerceptionCount=hIdCount[hIdInd][1]++;

				time = (int) (System.currentTimeMillis() / 1000);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotHandPerception"
							+ visualRobotHandPerceptionCount
							+ "' 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' 'http://knowrob.org/kb/knowrob.owl#VisualRobotHandPerception' h";
					assertTriple(assertString);
					// startTime
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotHandPerception"
							+ visualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#startTime' 'http://www.arbi.com/ontologies/arbi.owl#timepoint_"
							+ time + "' h";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotHandPerception"
							+ "' 'http://knowrob.org/kb/knowrob.owl#startTime' 'http://www.arbi.com/ontologies/arbi.owl#timepoint_"
							+ time + "' h";
					updateCurrentTriple(assertString);
					// objectActedOn
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotHandPerception"
							+ visualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#objectActedOn' 'http://www.arbi.com/ontologies/arbi.owl#"
							+ ID + "' h";
					assertTriple(assertString);
					// eventOccursAt
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotHandPerception"
							+ visualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#eventOccursAt' 'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_"
							+ ID + subVisualRobotHandPerceptionCount + "' h";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotHandPerception"
							+ "' 'http://knowrob.org/kb/knowrob.owl#eventOccursAt' 'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_"
							+ ID + "' h";
					updateCurrentTriple(assertString);
					// rotationMatrix3D
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' 'http://knowrob.org/kb/knowrob.owl#RotationMatrix3D' h";
					assertTriple(assertString);

					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + spaceName
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' 'http://knowrob.org/kb/knowrob.owl#RotationMatrix3D' h";
					assertTriple(assertString);
					//inFrontOf-Generally
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + spaceName 
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#inFrontOf-Generally' 'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount	+ "' h";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + spaceName 
							+ "' 'http://knowrob.org/kb/knowrob.owl#inFrontOf-Generally' 'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ "' h";
					updateCurrentTriple(assertString);
					// x,y,z
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m03' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ x + "')) h";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ "' 'http://knowrob.org/kb/knowrob.owl#m03' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ x + "')) h";
					updateCurrentTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m13' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ y + "')) h";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ "' 'http://knowrob.org/kb/knowrob.owl#m13' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ y + "')) h";
					updateCurrentTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m23' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ z + "')) h";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ "' 'http://knowrob.org/kb/knowrob.owl#m23' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ z + "')) h";
					updateCurrentTriple(assertString);
					// a,b,g
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m02' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ a + "')) h";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ "' 'http://knowrob.org/kb/knowrob.owl#m02' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ a + "')) h";
					updateCurrentTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m12' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ b + "')) h";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ "' 'http://knowrob.org/kb/knowrob.owl#m12' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ b + "')) h";
					updateCurrentTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m22' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ c + "')) h";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ "' 'http://knowrob.org/kb/knowrob.owl#m22' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ c + "')) h";
					updateCurrentTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m32' literal(type('http://www.w3.org/2001/XMLSchema#double','0')) h";
					assertTriple(assertString);

					// x,y,z
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + spaceName
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m03' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ finger_x + "')) h";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + spaceName
							+ "' 'http://knowrob.org/kb/knowrob.owl#m03' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ finger_x + "')) h";
					updateCurrentTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + spaceName
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m13' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ finger_y + "')) h";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + spaceName
							+ "' 'http://knowrob.org/kb/knowrob.owl#m13' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ finger_y + "')) h";
					updateCurrentTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + spaceName
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m23' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ finger_z + "')) h";
					assertTriple(assertString);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + spaceName
							+ "' 'http://knowrob.org/kb/knowrob.owl#m23' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ finger_z + "')) h";
					updateCurrentTriple(assertString);
					//sleep(20);

					if(visualRobotHandPerceptionCount%20==0){
						for(int i=visualRobotHandPerceptionInterval;i<visualRobotHandPerceptionCount-10;i++){
							retractTriple("'http://www.arbi.com/ontologies/arbi.owl#visualRobotHandPerception"+i+"' A"+" B h");
							//retractTriple("'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID+i+"' A"+" B");
						}
						if(visualRobotHandPerceptionInterval<visualRobotHandPerceptionCount-10)
							visualRobotHandPerceptionInterval=visualRobotHandPerceptionCount-10;
					//	sleep(5);
					}

					if(hIdCount[hIdInd][1]%removeTime==0){
						for(int i=hIdCount[hIdInd][0];i<hIdCount[hIdInd][1]-removeInterval;i++){
							retractTriple("'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID+i+"' A"+" B h");
							retractTriple("'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + spaceName+i+"' A"+" B h");
						}
						hIdCount[hIdInd][0]=hIdCount[hIdInd][1]-removeInterval;
					//	sleep(5);
					}

					sendMessage("hb");
				result = "[Perception_Manager/robot_hand_perception] time:"+time+
				"  Object_name:" + ID + ", Object Pose(X: " + x + " Y: "+ y+ " Z: " + z + "),Object Orientation(A: " + 
				a+ " B: " + b+ " G: " + c;
				// System.out.print(result);
			}
		},queueSize);

		try {
			sleep(500);
			MainServiceRequest mRequest = requestor.newMessage();
			List<String> temp = new ArrayList<String>();
			temp.add("init");
			int[] st = {99};
			mRequest.setPredicate(temp);
			mRequest.setStatus(st);

			temp = new ArrayList<String>();
			temp.add("ContextManager");
			mRequest.setManager(temp);
			requestor.publish(mRequest);
			sleep(1000);

		/*	while(true){
					MainServiceRequest request = requestor.newMessage();
					fsdProperty.add("");
					fsdProperty.add("currentObjectPose(Var1,Var2)");
					fsdProperty.add("currentRobotBodyPose(Var1,Var2)");
					fsdProperty.add("currentHandPose(Var1,Var2)");
					fsdProperty.add("empty_hand(Var1)");
					fsdProperty.add("opened_hand(Var1)");
					fsdProperty.add("closed_hand(Var1)");
					fsdProperty.add("graspedBy(Var1,Var2)");
					fsdProperty.add("on_Physical(Var1,Var2)");
					fsdProperty.add("misPlaced_item(Var1)");
					*/

/*
					for(int i=0;i<3;i++){
						fsdSubject.add("Var1");
						fsdObject.add("Var2");
						fsdStatus[i]=102;
					}

					for(int i=3;i<6;i++){
						fsdSubject.add("Var1");
						fsdObject.add("");
						fsdStatus[i]=102;
					}

					for(int i=6;i<8;i++){
						fsdSubject.add("Var1");
						fsdObject.add("Var2");
						fsdStatus[i]=102;
					}
					
					for(int i=8;i<9;i++){
						fsdSubject.add("Var1");
						fsdObject.add("");
						fsdStatus[i]=102;
					}

		*/
		/*
		  fsdStatus[0]=103;
			for(int i=1;i<10;i++){
						fsdStatus[i]=102;
					}
	
			request.setPredicate(fsdProperty);
			request.setParam1(fsdSubject);
			request.setParam2(fsdObject);
			request.setStatus(fsdStatus);
	
			
			request.setManager("ContextManager");
			requestor.publish(request);

			fsdSubject = new ArrayList<String>();
			fsdProperty = new ArrayList<String>();
			fsdObject = new ArrayList<String>();
			fsdStatus = new int[1000];
			
			sleep(3000);
			}*/
		} catch (Exception e) {
			e.printStackTrace();
		}

	}

	/*public void query(String param) {
		sleep(2);

		MonitorServiceRequest request = requestor.newMessage();
		ContextManager.setMessage(request, param);
		request.setManager("ContextManager");
		requestor.publish(request);
	}*/

	public void assertTriple(String triple) {
		String[] params = triple.split(" ");

		if(params[3].equals("o")){
			//System.out.println(oInd);
			oSubject.add(params[0]);
			oProperty.add(params[1]);
			oObject.add(params[2]);
			oStatus[oInd++]=3;
		}
		else if(params[3].equals("b")){
			bSubject.add(params[0]);
			bProperty.add(params[1]);
			bObject.add(params[2]);
			bStatus[bInd++]=3;
		}
		else if(params[3].equals("h")){
			hSubject.add(params[0]);
			hProperty.add(params[1]);
			hObject.add(params[2]);
			hStatus[hInd++]=3;
		}
		else if(params[3].equals("j")){
			jSubject.add(params[0]);
			jProperty.add(params[1]);
			jObject.add(params[2]);
			jStatus[jInd++]=3;
		}

	}

	public void retractTriple(String triple){
		String[] params = triple.split(" ");
		params[1]="P";
		params[2]="O";

		if(params[3].equals("o")){
			//System.out.println(oInd);
			oSubject.add(params[0]);
			oProperty.add(params[1]);
			oObject.add(params[2]);
			oStatus[oInd++]=4;
			//System.out.println(triple);
		}
		else if(params[3].equals("b")){
			bSubject.add(params[0]);
			bProperty.add(params[1]);
			bObject.add(params[2]);
			bStatus[bInd++]=4;
		}
		else if(params[3].equals("h")){
			hSubject.add(params[0]);
			hProperty.add(params[1]);
			hObject.add(params[2]);
			hStatus[hInd++]=4;
		}
		else if(params[3].equals("j")){
			jSubject.add(params[0]);
			jProperty.add(params[1]);
			jObject.add(params[2]);
			jStatus[jInd++]=4;
		}
	}

	public void updateCurrentTriple(String triple){
		retractTriple(triple);
		assertTriple(triple);
	}

	public void sendMessage(String str){
		MainServiceRequest request = requestor.newMessage();
		
		if(str.equals("ob")){
		//	oProperty[oInd]="End";
			request.setPredicate(oProperty);
			request.setParam1(oSubject);
			request.setParam2(oObject);
			request.setStatus(oStatus);
		//	oProperty[oInd]="";
			oInd=0;
			oSubject = new ArrayList<String>();
			oProperty = new ArrayList<String>();
	oObject = new ArrayList<String>();
	oStatus = new int[1000];
	//		oSubject.clear();// = new ArrayList<String>();
	//		oProperty.clear();// = new ArrayList<String>();
	//oObject.clear();// = new ArrayList<String>();
	//oStatus = new int[1000];
		}
		else if(str.equals("bb")){
		//	bProperty[bInd]="End";
			request.setPredicate(bProperty);
			request.setParam1(bSubject);
			request.setParam2(bObject);
			request.setStatus(bStatus);
	//		bProperty[bInd]="";
			bInd=0;
			bSubject = new ArrayList<String>();
			bProperty = new ArrayList<String>();
	bObject = new ArrayList<String>();
	bStatus = new int[1000];
	//		bSubject.clear();// = new ArrayList<String>();
	//		bProperty.clear();// = new ArrayList<String>();
	//bObject.clear();// = new ArrayList<String>();
	//bStatus = new int[1000];
		}
		else if(str.equals("hb")){
		//	hProperty[hInd]="End";
			request.setPredicate(hProperty);
			request.setParam1(hSubject);
			request.setParam2(hObject);
			request.setStatus(hStatus);
		//	hProperty[hInd]="";
			hInd=0;
			hSubject = new ArrayList<String>();
			hProperty = new ArrayList<String>();
	hObject = new ArrayList<String>();
	hStatus = new int[1000];
		//	hSubject.clear();// = new ArrayList<String>();
	//		hProperty.clear();// = new ArrayList<String>();
//	hObject.clear();// = new ArrayList<String>();
	//hStatus = new int[1000];
		}
		else if(str.equals("jb")){
		//	jProperty[jInd]="End";
			request.setPredicate(jProperty);
			request.setParam1(jSubject);
			request.setParam2(jObject);
			request.setStatus(jStatus);
		//	jProperty[jInd]="";
			jInd=0;
				jSubject = new ArrayList<String>();
			jProperty = new ArrayList<String>();
	jObject = new ArrayList<String>();
	jStatus = new int[1000];
			//jSubject.clear();// = new ArrayList<String>();
			//jProperty.clear();// = new ArrayList<String>();
//	jObject.clear();// = new ArrayList<String>();
	//jStatus = new int[1000];
		}

		List<String> temp = new ArrayList<String>();
		temp.add("ContextManager");
		request.setManager(temp);
		requestor.publish(request);
		//sleep(1);
	}

	public void sleep(int n){
		try{
			Thread.sleep(n);
		}catch(Exception e){}
	}

}
