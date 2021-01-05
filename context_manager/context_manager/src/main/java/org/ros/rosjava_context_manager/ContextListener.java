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
//import demo.*;

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

//import org.ros.rosjava_context_manager.demo_perception.Person;
//import rosjava_context_manager.Person;
/**
 * A simple {@link Subscriber} {@link NodeMain}.
 */
public class ContextListener extends AbstractNodeMain {
	int visualObjectPerceptionCount = 0;
	int visualRobotBodyPerceptionCount = 0;
	int visualRobotHandPerceptionCount = 0;
	int visualRobotLeftHandPerceptionCount = 0;
	int visualRobotRightHandPerceptionCount = 0;
	int jointPerceptionCount = 0;

	int visualObjectPerceptionInterval = 0;
	int visualRobotBodyPerceptionInterval = 0;
	int visualRobotHandPerceptionInterval = 0;
	int visualRobotLeftHandPerceptionInterval = 0;
	int visualRobotRightHandPerceptionInterval = 0;
	int jointPerceptionInterval = 0;

	int removeTime = 10;
	int removeInterval = 10;


	static List<String> oSubject = new ArrayList<String>();
	static List<String>  oProperty = new ArrayList<String>();
	static List<String>  oObject = new ArrayList<String>();
	static List<String>  oGraph = new ArrayList<String>();
	static int[] oStatus = new int[1000];
	static int oInd;
	static List<String>  oManager = new ArrayList<String>();


	static List<String>  bSubject = new ArrayList<String>();
	static List<String>  bProperty = new ArrayList<String>();
	static List<String>  bObject = new ArrayList<String>();
	static List<String>  bGraph = new ArrayList<String>();
	static int[] bStatus = new int[1000]; 
	static int bInd;
	static List<String>  bManager = new ArrayList<String>();

	static List<String>  hSubject = new ArrayList<String>();
	static List<String>  hProperty = new ArrayList<String>();
	static List<String>  hObject = new ArrayList<String>();
	static List<String>  hGraph = new ArrayList<String>();
	static int[] hStatus = new int[1000];
	static int hInd;
	static List<String>  hManager = new ArrayList<String>();

	static List<String>  jSubject = new ArrayList<String>();
	static List<String>  jProperty = new ArrayList<String>();
	static List<String>  jObject = new ArrayList<String>();
	static List<String>  jGraph = new ArrayList<String>();
	static int[] jStatus = new int[1000];
	static int jInd;
	static List<String>  jManager = new ArrayList<String>();

	static List<String> fsdSubject = new ArrayList<String>();
	static List<String> fsdProperty = new ArrayList<String>();
	static List<String> fsdObject = new ArrayList<String>();
	static int[] fsdStatus = new int[1000];
	static int fsdInd;
	static List<String> fsdManager = new ArrayList<String>();

	static int[][] oIdCount = new int[50][2];
	static int oIdInd;

	static int[][] bIdCount = new int[50][2];
	static int bIdInd;

	static int[][] hIdCount = new int[50][2];
	static int hIdInd;

	static int[][] jIdCount = new int[50][2];
	static int jIdInd;

	static List<String> obj_name = new ArrayList<String>();
	static List<String> obj_confidence = new ArrayList<String>();
	static List<String> affordance_name = new ArrayList<String>();
	static List<String> affordance_cofidence = new ArrayList<String>();

	final static int queueSize = 1000;

	Scanner scan;

	public static Publisher<QueryServiceRequest> questioner;
	static Publisher<MainServiceRequest> requestor;
	static Subscriber<MonitorServiceResponse> receiver;
	static Subscriber<QueryServiceResponse> answerer;
	// static Subscriber<Person> affordance_subscriber; jisu
	static Log log;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("context_listener");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		scan = new Scanner(System.in);
		log = connectedNode.getLog();

		Subscriber<std_msgs.Float32MultiArray> object_subscriber = connectedNode.newSubscriber("/objects_infos",
				std_msgs.Float32MultiArray._TYPE);// Subscriber node --> object
		Subscriber<std_msgs.Float32MultiArray> visual_robot_subscriber = connectedNode
				.newSubscriber("/visual_robot_perceptions", std_msgs.Float32MultiArray._TYPE);// Subscriber node -->
																								// object
		Subscriber<Person> affordance_subscriber = connectedNode.newSubscriber("/affordance_info", Person._TYPE);				//jisu																				

		// /j2n6s300
		Subscriber<sensor_msgs.JointState> robot_joint_subscriber = connectedNode
				.newSubscriber("/joint_statess", sensor_msgs.JointState._TYPE);
		//receiver = connectedNode.newSubscriber("ContextManager/Monitor/ProvisionForPM", MonitorServiceResponse._TYPE);
		answerer = connectedNode.newSubscriber("context_manager/query/provision_for_pm", QueryServiceResponse._TYPE);

		questioner = connectedNode.newPublisher("context_manager/query/reception", QueryServiceRequest._TYPE);
		requestor = connectedNode.newPublisher("context_manager/main/reception", MainServiceRequest._TYPE);


		
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
		});

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
			for(int k=0;k<message.getData().length;k+=11){
				int subVisualObjectPerceptionCount=0;
				String result = "";
				String assertString = "";
				String retractString = "";
				float x = message.getData()[0+k];
				float y = message.getData()[1+k];
				float z = message.getData()[2+k];
				float a = message.getData()[3+k];
				float b = message.getData()[4+k];
				float c = message.getData()[5+k];
				float d = message.getData()[6+k];
				float width = message.getData()[7+k];
				float depth = message.getData()[8+k];
				float height = message.getData()[9+k];
				float IDN = message.getData()[10+k];
				String ID = "";
				String type = "";
				int time = 0;

				// radian to degree
				//a = (float) Math.toDegrees(a);
				//b = (float) Math.toDegrees(b);
				//c = (float) Math.toDegrees(c);

				if (IDN == (float) 0) {
					ID = "milk";
					oIdInd=0;
				} else if (IDN == (float) 1) {
					ID = "juice";
					oIdInd=1;
				} else if (IDN == (float) 2) {
					ID = "table";
					oIdInd=2;
				} else if (IDN == (float) 3) {
					ID = "red_gotica";
					oIdInd=3;
				} else if (IDN == (float) 4) {
					ID = "bakey";
					oIdInd=4;
				} else if (IDN == (float) 5) {
					ID = "pringles_onion";
					oIdInd=5;
				} else if (IDN == (float) 6) {
					ID = "diget_box";
					oIdInd=6;
				} else if (IDN == (float) 7) {
					ID = "diget";
					oIdInd=7;
				} else if (IDN == (float) 8) {
					ID = "gotica";
					oIdInd=8;
				} else if (IDN == (float) 9) {
					ID = "vitamin_water";
					oIdInd=9;
				} else if (IDN == (float) 10) {
					ID = "diget_small_box";
					oIdInd=10;
				} else if (IDN == (float) 11) {
					ID = "chocolate_syrup";
					oIdInd=11;
				} else if (IDN == (float) 12) {
					ID = "orange_cup";
					oIdInd=12;
				} else if (IDN == (float) 13) {
					ID = "orange_juice";
					oIdInd=13;
				} else if (IDN == (float) 14) {
					ID = "mug_red";
					oIdInd=14;
				} else if (IDN == (float) 15) {
					ID = "mug_white";
					oIdInd=15;
				} else if (IDN == (float) 16) {
					ID = "corn_flight";
					oIdInd=16;
				} else if (IDN == (float) 17) {
					ID = "telephone";
					oIdInd=17;
				} else if (IDN == (float) 18) {
					ID = "calculator";
					oIdInd=18;
				} else if (IDN == (float) 19) {
					ID = "black_bowl";
					oIdInd=19;
				} else if (IDN == (float) 20) {
					ID = "handcream";
					oIdInd=20;
				} else if (IDN == (float) 21) {
					ID = "small_milk";
					oIdInd=21;
				} else if (IDN == (float) 22) {
					ID = "red_cup";
					oIdInd=22;
				} else if (IDN == (float) 23) {
					ID = "elephant_mug";
					oIdInd=23;
				} else if (IDN == (float) 24) {
					ID = "kettle";
					oIdInd=24;
				} else if (IDN == (float) 25) {
					ID = "eiffel_tower_mug";
					oIdInd=25;
				} 
				/*else if (IDN == (float) 102) {
					ID = "chips_can_3";
					oIdInd=2;
				} 
				else if (IDN == (float) 103) {
					ID = "coffee_can_1";
					oIdInd=3;
				} else if (IDN == (float) 104) {
					ID = "coffee_can_2";
					oIdInd=4;
				} else if (IDN == (float) 105) {
					ID = "cracker_box_1";
					oIdInd=5;
				} else if (IDN== (float) 106) {
					ID = "cracker_box_2";
					oIdInd=6;
				} else if (IDN == (float) 107) {
					ID = "cracker_box_3";
					oIdInd=7;
				} else if (IDN == (float) 108) {
					ID = "jelly_box_1";
					oIdInd=8;
				} else if (IDN == (float) 109) {
					ID = "jelly_box_2";
					oIdInd=9;
				} else if (IDN == (float) 110) {
					ID = "sugar_box_1";
					oIdInd=10;
				} else if (IDN == (float) 111) {
					ID = "sugar_box_2";
					oIdInd=11;
				}else if (IDN == (float) 112) {
					ID = "sugar_box_3";
					oIdInd=12;
				}  else if (IDN == (float) 113) {
					ID = "tomato_soup_1";
					oIdInd=13;
				} else if (IDN == (float) 114) {
					ID = "tomato_soup_2";
					oIdInd=14;
				}else if (IDN == (float) 115) {
					ID = "tomato_soup_3";
					oIdInd=15;
				}*/
				else {
					oIdInd=49;
					continue;
				}

				result += ID + " " + x + " " + y + " " + z + " " + a + " " + b + " " + c + " " + d + " " + width + " " + depth + " " + height;
				//System.out.println(result);
				

				//visualObjectPerceptionCount++;//=oIdCount[oIdInd][1]++;
				subVisualObjectPerceptionCount=oIdCount[oIdInd][1]++;
				visualObjectPerceptionCount=subVisualObjectPerceptionCount;

				time = (int) (System.currentTimeMillis() / 1000);//currentTimeMillis() : 1/1000 s return --> 0m 1s = 100 --> 1s  
	

				retractString = "arbi:"+ID+" knowrob:widthOfObject A objectPerception o";
				retractTriple(retractString);
				assertString = "arbi:"+ID+" knowrob:widthOfObject literal(type(xsd,'" + width + "'))"
				+ " objectPerception o";
				assertTriple(assertString);

				retractString = "arbi:"+ID+" knowrob:depthOfObject A objectPerception o";
				retractTriple(retractString);
				assertString = "arbi:"+ID+" knowrob:depthOfObject literal(type(xsd,'" + depth + "'))"
				+ " objectPerception o";
				assertTriple(assertString);

				retractString = "arbi:"+ID+" knowrob:heightOfObject A objectPerception o";
				retractTriple(retractString);
				assertString = "arbi:"+ID+" knowrob:heightOfObject literal(type(xsd,'" + height + "'))"
				+ " objectPerception o";
				assertTriple(assertString);

				assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception" +"_"+ID+"_"
						+ visualObjectPerceptionCount
						+ "' 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' 'http://knowrob.org/kb/knowrob.owl#VisualObjectPerception'"
						+ " objectPerception o";

				assertTriple(assertString);
				// startTime
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception"+"_"+ID+"_"
						+ visualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#startTime' 'http://www.arbi.com/ontologies/arbi.owl#timepoint_"
						+ time +"'"
						+ " objectPerception o";

				assertTriple(assertString);

				
				// objectActedOn
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception"+"_"+ID+"_"
						+ visualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#objectActedOn' 'http://www.arbi.com/ontologies/arbi.owl#"
						+ ID+"'"
						+ " objectPerception o";

				assertTriple(assertString);
				// eventOccursAt
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception"+"_"+ID+"_"
						+ visualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#eventOccursAt' 'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_"
						+ ID + subVisualObjectPerceptionCount+"'"
						+ " objectPerception o";

				assertTriple(assertString);

				
				// rotationMatrix3D
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ subVisualObjectPerceptionCount
						+ "' 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' 'http://knowrob.org/kb/knowrob.owl#RotationMatrix3D'"
						+ " objectPerception o";

				assertTriple(assertString);
				// x,y,z
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ subVisualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#m03' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ x + "'))"
						+ " objectPerception o";
						
				assertTriple(assertString);
				
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ subVisualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#m13' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ y + "'))"
						+ " objectPerception o";

				assertTriple(assertString);
			
			
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ subVisualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#m23' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ z + "'))"
						+ " objectPerception o";

				assertTriple(assertString);
			
				// a,b,g
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ subVisualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#m02' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ a + "'))"
						+ " objectPerception o";
						
				assertTriple(assertString);
				
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ subVisualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#m12' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ b + "'))" 
						+ " objectPerception o";

				assertTriple(assertString);
				
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ subVisualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#m22' literal(type('http://www.w3.org/2001/XMLSchema#double','"
						+ c + "'))"
						+ " objectPerception o";
						
				assertTriple(assertString);
				
				assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
						+ subVisualObjectPerceptionCount
						+ "' 'http://knowrob.org/kb/knowrob.owl#m32' literal(type('http://www.w3.org/2001/XMLSchema#double','0'))"
						+ " objectPerception o";
						
				assertTriple(assertString);
				
				//test
				//assertString = ID
				//		+ " currentObjectPose a o";
				//updateCurrentTriple(assertString);
				// print data
				/*
				result = "[Perception_Manager/visual_object_perception] time:"+time+
				"  Object_name:" + ID + ", Object Pose(X: " + x + " Y: "+ y+ " Z: " + z + "),Object Orientation(A: " + 
				a+ " B: " + b+ " G: " + c;
				System.out.println(result);
				*/
				
				/*


				if(visualObjectPerceptionCount%removeTime==0){
					//System.out.println(visualObjectPerceptionCount);
						for(int i=visualObjectPerceptionInterval;i<visualObjectPerceptionCount-removeInterval;i++){
							retractTriple("'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception"+"_"+ID+"_"+i+"' A"+" B o");
							//retractTriple("'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID+i+"' A"+" B o");
						}
							visualObjectPerceptionInterval=visualObjectPerceptionCount-removeInterval;
					//	sleep(5);
					}
					*/
					if(oIdCount[oIdInd][1]%removeTime==0){
						for(int i=oIdCount[oIdInd][0];i<oIdCount[oIdInd][1]-removeInterval;i++){
							retractTriple("'http://www.arbi.com/ontologies/arbi.owl#visualObjectPerception"+"_"+ID+"_"+i+"' A"+" B C o");

							retractTriple("'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID+i+"' A"+" B C o");
						}
						oIdCount[oIdInd][0]=oIdCount[oIdInd][1]-removeInterval;
					
					}

			}

					sendMessage("ob");
			}
		},queueSize);

		// VisualRobotPerception
		visual_robot_subscriber.addMessageListener(new MessageListener<std_msgs.Float32MultiArray>() {
			@Override
			public void onNewMessage(std_msgs.Float32MultiArray message) {
				if(message.getData().length==0)return;

				int subVisualRobotBodyPerceptionCount=0;
				int subVisualRobotHandPerceptionCount=0;
				String result = "";
				String assertString = "";
				float x = message.getData()[0];
				float y = message.getData()[1];
				float z = message.getData()[2];
				float a = message.getData()[3];
				float b = message.getData()[4];
				float c = message.getData()[5];
				float depth = message.getData()[6];
				float width = message.getData()[7];
				float height = message.getData()[8];
				float id = message.getData()[9];

				/*
				float finger_x = message.getData()[7];
				float finger_y = message.getData()[8];
				float finger_z = message.getData()[9];
				*/

				String ID = "";
				String spaceName="space_surrounded_by_hand";
				String type = "";
				int time = 0;

				// radian to degree
				//a = (float) Math.toDegrees(a);
				//b = (float) Math.toDegrees(b);
				//c = (float) Math.toDegrees(c);


				if (id == (float) 10) {
					ID = "socialrobot";//"hubo_1";//"robot_body"
					bIdInd=0;
				} else if (id == (float) 12) {
					ID = "left_hand_1";//"hubo_left_hand_1";//robot_hand";
					hIdInd=0;
				} else if (id == (float) 11) {
					ID = "right_hand_1";//"robot_finger_mid";
					hIdInd=1;
				} else {
					bIdInd=49;
					hIdInd=49;
				}
				time = (int) (System.currentTimeMillis() / 1000);


				// assert data
				if (ID.equals("socialrobot")) {
					visualRobotBodyPerceptionCount++;
					subVisualRobotBodyPerceptionCount=bIdCount[bIdInd][1]++;

					time = (int) (System.currentTimeMillis() / 1000);

						assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotBodyPerception"
								+ visualRobotBodyPerceptionCount
								+ "' 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' 'http://knowrob.org/kb/knowrob.owl#VisualRobotBodyPerception'"
								+ " robotPerception b";

						assertTriple(assertString);
						// startTime
						assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotBodyPerception"
								+ visualRobotBodyPerceptionCount
								+ "' 'http://knowrob.org/kb/knowrob.owl#startTime' 'http://www.arbi.com/ontologies/arbi.owl#timepoint_"
								+ time + "'"
								+ " robotPerception b";

						assertTriple(assertString);
						
						// objectActedOn
						assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotBodyPerception"
								+ visualRobotBodyPerceptionCount
								+ "' 'http://knowrob.org/kb/knowrob.owl#objectActedOn' 'http://www.arbi.com/ontologies/arbi.owl#"
								+ ID + "'"
								+ " robotPerception b";

						assertTriple(assertString);
						// eventOccursAt
						assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotBodyPerception"
								+ visualRobotBodyPerceptionCount
								+ "' 'http://knowrob.org/kb/knowrob.owl#eventOccursAt' 'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_"
								+ ID + subVisualRobotBodyPerceptionCount + "'"
								+ " robotPerception b";

						assertTriple(assertString);
						
						// rotationMatrix3D
						assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
								+ subVisualRobotBodyPerceptionCount
								+ "' 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' 'http://knowrob.org/kb/knowrob.owl#RotationMatrix3D'"
								+ " robotPerception b";

						assertTriple(assertString);
						// x,y,z
						assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
								+ subVisualRobotBodyPerceptionCount
								+ "' 'http://knowrob.org/kb/knowrob.owl#m03' literal(type('http://www.w3.org/2001/XMLSchema#double','"
								+ x + "'))"
								+ " robotPerception b";

						assertTriple(assertString);
						
						assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
								+ subVisualRobotBodyPerceptionCount
								+ "' 'http://knowrob.org/kb/knowrob.owl#m13' literal(type('http://www.w3.org/2001/XMLSchema#double','"
								+ y + "'))"
								+ " robotPerception b";

						assertTriple(assertString);
						
						assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
								+ subVisualRobotBodyPerceptionCount
								+ "' 'http://knowrob.org/kb/knowrob.owl#m23' literal(type('http://www.w3.org/2001/XMLSchema#double','"
								+ z + "'))"
								+ " robotPerception b";

						assertTriple(assertString);
						
						// a,b,g
						assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
								+ subVisualRobotBodyPerceptionCount
								+ "' 'http://knowrob.org/kb/knowrob.owl#m02' literal(type('http://www.w3.org/2001/XMLSchema#double','"
								+ a + "'))"
								+ " robotPerception b";

						assertTriple(assertString);
						
						assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
								+ subVisualRobotBodyPerceptionCount
								+ "' 'http://knowrob.org/kb/knowrob.owl#m12' literal(type('http://www.w3.org/2001/XMLSchema#double','"
								+ b + "'))"
								+ " robotPerception b";

						assertTriple(assertString);
						
						assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
								+ subVisualRobotBodyPerceptionCount
								+ "' 'http://knowrob.org/kb/knowrob.owl#m22' literal(type('http://www.w3.org/2001/XMLSchema#double','"
								+ c + "'))"
								+ " robotPerception b";

						assertTriple(assertString);
						
						assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
								+ subVisualRobotBodyPerceptionCount
								+ "' 'http://knowrob.org/kb/knowrob.owl#m32' literal(type('http://www.w3.org/2001/XMLSchema#double','0'))"
								+ " robotPerception b";

						assertTriple(assertString);

						//sleep(20);

						/*

						if(visualRobotBodyPerceptionCount%removeTime==0){
							for(int i=visualRobotBodyPerceptionInterval;i<visualRobotBodyPerceptionCount-10;i++){
								retractTriple("'http://www.arbi.com/ontologies/arbi.owl#visualRobotBodyPerception"+i+"' A"+" B b");
								//retractTriple("'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID+i+"' A"+" B");
							}
							if(visualRobotBodyPerceptionInterval<visualRobotBodyPerceptionCount-10)
								visualRobotBodyPerceptionInterval=visualRobotBodyPerceptionCount-10;
						
						//	sleep(5);
						}

						*/

						if(bIdCount[bIdInd][1]%removeTime==0){
							for(int i=bIdCount[bIdInd][0];i<bIdCount[bIdInd][1]-removeInterval;i++){
								retractTriple("'http://www.arbi.com/ontologies/arbi.owl#visualRobotBodyPerception"+i+"' A"+" B C b");

								retractTriple("'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID+i+"' A"+" B C b");
							}
							bIdCount[bIdInd][0]=bIdCount[bIdInd][1]-removeInterval;
						//	sleep(5);
						}

						sendMessage("bb");
						/*
					result = "[Perception_Manager/robot_body_perception] time:"+time+
					"  Object_name:" + ID + ", Object Pose(X: " + x + " Y: "+ y+ " Z: " + z + "),Object Orientation(A: " + 
					a+ " B: " + b+ " G: " + c;
					 System.out.println(result);*/
					 
				} else if (ID.equals("left_hand_1") || ID.equals("right_hand_1")) {
					
				if (ID.equals("left_hand_1"))
				visualRobotHandPerceptionCount=visualRobotLeftHandPerceptionCount++;
				else if (ID.equals("right_hand_1"))
				visualRobotHandPerceptionCount=visualRobotRightHandPerceptionCount++;
				subVisualRobotHandPerceptionCount=hIdCount[hIdInd][1]++;

				time = (int) (System.currentTimeMillis() / 1000);
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotHandPerception" +"_"+ID+"_"
							+ visualRobotHandPerceptionCount
							+ "' 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' 'http://knowrob.org/kb/knowrob.owl#VisualRobotHandPerception'"
							+ " robotPerception h";

					assertTriple(assertString);
					// startTime
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotHandPerception" +"_"+ID+"_"
							+ visualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#startTime' 'http://www.arbi.com/ontologies/arbi.owl#timepoint_"
							+ time + "'"
							+ " robotPerception h";

					assertTriple(assertString);
					
					// objectActedOn
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotHandPerception" +"_"+ID+"_"
							+ visualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#objectActedOn' 'http://www.arbi.com/ontologies/arbi.owl#"
							+ ID + "'"
							+ " robotPerception h";

					assertTriple(assertString);
					// eventOccursAt
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#visualRobotHandPerception" +"_"+ID+"_"
							+ visualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#eventOccursAt' 'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_"
							+ ID + subVisualRobotHandPerceptionCount + "'"
							+ " robotPerception h";

					assertTriple(assertString);
					
					// rotationMatrix3D
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' 'http://knowrob.org/kb/knowrob.owl#RotationMatrix3D'"
							+ " robotPerception h";

					assertTriple(assertString);

					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + spaceName
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' 'http://knowrob.org/kb/knowrob.owl#RotationMatrix3D'"
							+ " robotPerception h";

					assertTriple(assertString);
					//inFrontOf-Generally
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + spaceName 
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#inFrontOf-Generally' 'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount	+ "'"
							+ " robotPerception h";

					assertTriple(assertString);
					
					// x,y,z
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m03' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ x + "'))"
							+ " robotPerception h";

					assertTriple(assertString);
					
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m13' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ y + "'))"
							+ " robotPerception h";

					assertTriple(assertString);
					
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m23' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ z + "'))"
							+ " robotPerception h";

					assertTriple(assertString);
			
					// a,b,g
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m02' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ a + "'))"
							+ " robotPerception h";

					assertTriple(assertString);
					
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m12' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ b + "'))"
							+ " robotPerception h";

					assertTriple(assertString);
					
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m22' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ c + "'))"
							+ " robotPerception h";

					assertTriple(assertString);
					
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m32' literal(type('http://www.w3.org/2001/XMLSchema#double','0'))"
							+ " robotPerception h";

					assertTriple(assertString);



					


				}else if (ID.equals("robot_finger_mid")) {

					subVisualRobotHandPerceptionCount=hIdCount[hIdInd][1];

					// x,y,z
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + spaceName
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m03' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ x + "'))"
							+ " robotPerception h";

					assertTriple(assertString);
					
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + spaceName
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m13' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ y + "'))"
							+ " robotPerception h";

					assertTriple(assertString);
					
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + spaceName
							+ subVisualRobotHandPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#m23' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ z + "'))"
							+ " robotPerception h";

					assertTriple(assertString);
				}
					
					//sleep(20);
					/*
					if (ID.equals("hubo_left_hand_1"))
				visualRobotHandPerceptionInterval=visualRobotLeftHandPerceptionInterval;
				else if (ID.equals("hubo_right_hand_1"))
				visualRobotHandPerceptionInterval=visualRobotRightHandPerceptionInterval;


					if(visualRobotHandPerceptionCount%removeTime==0){
						for(int i=visualRobotHandPerceptionInterval;i<visualRobotHandPerceptionCount-removeInterval;i++){
							retractTriple("'http://www.arbi.com/ontologies/arbi.owl#visualRobotHandPerception" +"_"+ID+"_"+i+"' A"+" B h");
							//retractTriple("'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID+i+"' A"+" B");
						}
						if(visualRobotHandPerceptionInterval<visualRobotHandPerceptionCount-removeInterval){
							visualRobotHandPerceptionInterval=visualRobotHandPerceptionCount-removeInterval;
							if (ID.equals("hubo_left_hand_1"))
				visualRobotLeftHandPerceptionInterval=visualRobotHandPerceptionInterval;
				else if (ID.equals("hubo_right_hand_1"))
				visualRobotRightHandPerceptionInterval=visualRobotHandPerceptionInterval;
						}

					//	sleep(5);
					}*/
					

					if(hIdCount[hIdInd][1]%removeTime==0){
						for(int i=hIdCount[hIdInd][0];i<hIdCount[hIdInd][1]-removeInterval;i++){
							retractTriple("'http://www.arbi.com/ontologies/arbi.owl#visualRobotHandPerception" +"_"+ID+"_"+i+"' A"+" B C h");

							retractTriple("'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + ID+i+"' A"+" B C h");
							retractTriple("'http://www.arbi.com/ontologies/arbi.owl#rotationMatrix3D_" + spaceName+i+"' A"+" B C h");
						}
						hIdCount[hIdInd][0]=hIdCount[hIdInd][1]-removeInterval;
					//	sleep(5);
					}

					
					sendMessage("hb");
				
				 
			
				
				result = "[Perception_Manager/robot_hand_perception] time:"+time+
				"  Object_name:" + ID + ", Object Pose(X: " + x + " Y: "+ y+ " Z: " + z + "),Object Orientation(A: " + 
				a+ " B: " + b+ " G: " + c;
				 System.out.println(result);
				 
				 }
		});

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
							+ "' 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type' 'http://knowrob.org/kb/knowrob.owl#JointPerception'"
							+ " graspPerception j";

					assertTriple(assertString);
					// startTime
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#jointPerception" + jointPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#startTime' 'http://www.arbi.com/ontologies/arbi.owl#timepoint_"
							+ time + "'"
							+ " graspPerception j";

					assertTriple(assertString);
				
					// objectActedOn
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#jointPerception" + jointPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#objectActedOn' 'http://www.arbi.com/ontologies/arbi.owl#"
							+ name + "'"
							+ " graspPerception j";

					assertTriple(assertString);
					// radius
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#jointPerception" + jointPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#radius' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ position + "'))"
							+ " graspPerception j";

					assertTriple(assertString);
					
					// velocity
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#jointPerception" + jointPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#velocity' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ velocity + "'))"
							+ " graspPerception j";

					assertTriple(assertString);
					
					// effort
					assertString = "'http://www.arbi.com/ontologies/arbi.owl#jointPerception" + jointPerceptionCount
							+ "' 'http://knowrob.org/kb/knowrob.owl#effort' literal(type('http://www.w3.org/2001/XMLSchema#double','"
							+ effort + "'))"
							+ " graspPerception j";

					assertTriple(assertString);
					

					
					// print data
					
					result_joint = "[Perception_Manager/jointPerception] " + "time:"+time+", Joint Name:" + name + ", Joint radius:("+position+"), Joint Velocity("+velocity+"), Joint Effort:("+effort+")";
					//System.out.println(result_joint);
					 
					// sleep(15);
				


				}
				
				int tmps = 5;
				if(jointPerceptionCount%(6*tmps)==0){ //message.getName().size()
						for(int k=jointPerceptionInterval;k<jointPerceptionCount-tmps*6;k++)
							retractTriple("'http://www.arbi.com/ontologies/arbi.owl#jointPerception"+k+"' A"+" B C j");
						jointPerceptionInterval=jointPerceptionCount-tmps*6;
					//	sleep(5);
					}
					
					sendMessage("jb");
			
			
			}
		},queueSize);

		affordance_subscriber.addMessageListener(new MessageListener<Person>() {

			@Override
			public void onNewMessage(Person message) {
				//int time = message.getHeader().getStamp().secs;
				//String model;
				System.out.println("!!!!!!!!!!!!!!!!!!!!!!!1");
				for(int i = 0; i < message.getObjName().size(); i++){
					System.out.println("~~~~~~~~~~"+message.getObjName().get(i));
				}
				//System.out.println("message!~~~~"+message.getModel());

			}
		},queueSize);


try {
			sleep(500);
			MainServiceRequest mRequest = requestor.newMessage();
			List<String> req = new ArrayList<String>();
			req.add("init");
			int[] st = {99};
			mRequest.setPredicate(req);
			mRequest.setStatus(st);

			req = new ArrayList<String>();
			req.add("ContextManager");
			mRequest.setManager(req);
			req = new ArrayList<String>();
			req.add("social_robot"); //scan.next());
			mRequest.setParam1(req);

			requestor.publish(mRequest);
			sleep(1000);
}catch(Exception e)
{
		e.printStackTrace();
}



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

	}


	/*
	 * public void query(String param) { sleep(2);
	 * 
	 * MonitorServiceRequest request = requestor.newMessage();
	 * ContextManager.setMessage(request, param);
	 * request.setManager("ContextManager"); requestor.publish(request); }
	 */

	public void assertTriple(String triple) {
		String[] params = triple.split(" ");

		if(params[4].equals("o")){
			//System.out.println(oInd);
			oSubject.add(params[0]);
			oProperty.add(params[1]);
			oObject.add(params[2]);
			oGraph.add(params[3]);
			oStatus[oInd++]=3;
			oManager.add("ContextManager");
		}
		else if(params[4].equals("b")){
			bSubject.add(params[0]);
			bProperty.add(params[1]);
			bObject.add(params[2]);
			bGraph.add(params[3]);
			bStatus[bInd++]=3;
			bManager.add("ContextManager");
		}
		else if(params[4].equals("h")){
			hSubject.add(params[0]);
			hProperty.add(params[1]);
			hObject.add(params[2]);
			hGraph.add(params[3]);
			hStatus[hInd++]=3;
			hManager.add("ContextManager");
		}
		else if(params[4].equals("j")){
			jSubject.add(params[0]);
			jProperty.add(params[1]);
			jObject.add(params[2]);
			jGraph.add(params[3]);
			jStatus[jInd++]=3;
			jManager.add("ContextManager");
		}
	}

	public void retractTriple(String triple){
		String[] params = triple.split(" ");
		//params[0]=params[0];
		//params[1]="P";
		//params[2]="O";
		params[3]="";

		if(params[4].equals("o")){
			//System.out.println(oInd);
			oSubject.add(params[0]);
			oProperty.add(params[1]);
			oObject.add(params[2]);
			oGraph.add(params[3]);
			oStatus[oInd++]=4;
			oManager.add("ContextManager");
		}
		else if(params[4].equals("b")){
			bSubject.add(params[0]);
			bProperty.add(params[1]);
			bObject.add(params[2]);
			bGraph.add(params[3]);
			bStatus[bInd++]=4;
			bManager.add("ContextManager");
		}
		else if(params[4].equals("h")){
			hSubject.add(params[0]);
			hProperty.add(params[1]);
			hObject.add(params[2]);
			hGraph.add(params[3]);
			hStatus[hInd++]=4;
			hManager.add("ContextManager");
		}
		else if(params[4].equals("j")){
			jSubject.add(params[0]);
			jProperty.add(params[1]);
			jObject.add(params[2]);
			jGraph.add(params[3]);
			jStatus[jInd++]=4;
			jManager.add("ContextManager");
		}
	}

	public void updateTriple(String triple) {
		retractTriple(triple);
		assertTriple(triple);
	}

	public void sendMessage(String str) {
		MainServiceRequest request = requestor.newMessage();

		if (str.equals("ob")) {
			// oProperty[oInd]="End";
			request.setPredicate(oProperty);
			request.setParam1(oSubject);
			request.setParam2(oObject);
			request.setParam4(oGraph);
			request.setStatus(oStatus);
			request.setManager(oManager);
			// oProperty[oInd]="";
			oInd = 0;
			oSubject = new ArrayList<String>();
			oProperty = new ArrayList<String>();
			oObject = new ArrayList<String>();
			oGraph = new ArrayList<String>();
			oStatus = new int[1000];
			oManager = new ArrayList<String>();
		} else if (str.equals("bb")) {

			request.setPredicate(bProperty);
			request.setParam1(bSubject);
			request.setParam2(bObject);
			request.setParam4(bGraph);
			request.setStatus(bStatus);
			request.setManager(bManager);

			bInd = 0;
			bSubject = new ArrayList<String>();
			bProperty = new ArrayList<String>();
			bObject = new ArrayList<String>();
			bGraph = new ArrayList<String>();
			bStatus = new int[1000];
			bManager = new ArrayList<String>();
		} else if (str.equals("hb")) {

			request.setPredicate(hProperty);
			request.setParam1(hSubject);
			request.setParam2(hObject);
			request.setParam4(hGraph);
			request.setStatus(hStatus);
			request.setManager(hManager);

			hInd = 0;
			hSubject = new ArrayList<String>();
			hProperty = new ArrayList<String>();
			hObject = new ArrayList<String>();
			hGraph = new ArrayList<String>();
			hStatus = new int[1000];
			hManager = new ArrayList<String>();
		} else if (str.equals("jb")) {

			request.setPredicate(jProperty);
			request.setParam1(jSubject);
			request.setParam2(jObject);
			request.setParam4(jGraph);
			request.setStatus(jStatus);
			request.setManager(jManager);

			jInd = 0;
			jSubject = new ArrayList<String>();
			jProperty = new ArrayList<String>();
			jObject = new ArrayList<String>();
			jGraph = new ArrayList<String>();
			jStatus = new int[1000];
			jManager = new ArrayList<String>();
		}

		requestor.publish(request);
		// sleep(1);
	}

	public void sleep(int n) {
		try {
			Thread.sleep(n);
		} catch (Exception e) {
		}
	}

}
