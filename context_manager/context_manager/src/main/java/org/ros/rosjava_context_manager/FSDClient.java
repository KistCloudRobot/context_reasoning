/*
     * Copyright (C) 2011 Google Inc.
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

    import java.lang.*;
    import java.util.*;
    import java.lang.Integer;
    import org.ros.exception.RemoteException;
    import org.ros.exception.RosRuntimeException;
    import org.ros.exception.ServiceNotFoundException;
    import org.ros.namespace.GraphName;
    import org.ros.node.AbstractNodeMain;
    import org.ros.node.ConnectedNode;
    import org.ros.node.NodeMain;
    import org.ros.node.service.ServiceClient;
    import org.ros.node.service.ServiceResponseBuilder;
    import org.ros.node.service.ServiceResponseListener;
    import rosjava_custom_srv.*;
    import org.apache.commons.logging.Log;
    import org.ros.message.MessageListener;
    import org.ros.node.topic.*;
    import org.ros.internal.message.action.*;
    import org.jpl7.*;
    import java.util.*;
    
    /**
     * A simple {@link ServiceClient} {@link NodeMain}. The original code is created
     * by:
     *
     * @author damonkohler@google.com (Damon Kohler) The custom implementation is
     *         created by v.s.moisiadis@gmail.com(Vasileios Moisiadis)
     */
    public class FSDClient extends AbstractNodeMain {
      Scanner scan = new Scanner(System.in);
      //static Subscriber<PredicateServiceRequest> receptionistForPredicate;
      static Publisher<MonitorServiceRequest> requestor;
      static Subscriber<MonitorServiceResponse> receiver;
      static Subscriber<Monitor> mReceiver;
      //static ServiceClient<MonitorServiceRequest, MonitorServiceResponse> serviceClient;
    
      final static int queueSize = 1000;
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("rosjava_tutorial_custom_custom_services/fsd_client");
      }
    
      /*
      public void setMessage(MonitorServiceRequest request, String param) {
        String[] params = param.split(" ");
    
        request.setPredicate(params[0]);
        request.setParam1(params[1]);
        request.setParam2(params[2]);
        request.setParam3(params[3]);
        request.setParam4(params[4]);
        request.setStatus(Integer.parseInt(params[5]));
      }*/
    
      @Override
      public void onStart(final ConnectedNode connectedNode) {
        Scanner queryInput = new Scanner(System.in);
        
        try {
         // serviceClient = connectedNode.newServiceClient("ContextManager/MonitorService", MonitorService._TYPE);
         //receptionistForPredicate = connectedNode.newPublisher("ContextManager/Predicate/Reception", PredicateServiceRequest._TYPE);
         requestor = connectedNode.newPublisher("context_manager/monitor/reception", MonitorServiceRequest._TYPE);
         // add here##############################
	 //requestor = connectedNode.newPublisher("save_data", MonitorServiceRequest._TYPE);
         receiver = connectedNode.newSubscriber("context_manager/monitor/provision_for_tm",MonitorServiceResponse._TYPE);
         mReceiver = connectedNode.newSubscriber("context_manager/monitor",Monitor._TYPE);
         //receiver = connectedNode.newSubscriber("ContextManager/Monitor/Provision", MonitorServiceResponse._TYPE);
        } catch (Exception e) {
          throw new RosRuntimeException(e);
        }

        //test
        receiver.addMessageListener(new MessageListener<MonitorServiceResponse>() {
          @Override
          public void onNewMessage(MonitorServiceResponse response) {
            
            String ans = Parser.text_transPredicate(response);
            connectedNode.getLog().info(String.format("The response is : "+ans));
          }
        }, queueSize);

          //test
        mReceiver.addMessageListener(new MessageListener<Monitor>() {
          @Override
          public void onNewMessage(Monitor response) {
            
            String ans = response.getPredicate() + " " + response.getParam1() + " " + response.getParam2() + " " +response.getParam3() + " " + response.getParam4();//Parser.text_transPredicate(response);
            connectedNode.getLog().info(String.format("The response is : "+ans));
          }
        }, queueSize);


        /*
        final PredicateServiceRequest request = receptionistForPredicate.newMessage();
        request.setPredicate("currentObjectPerception");
        request.setParam1("Object");
        request.setParam2("Perception");
        request.setParam3("0");
        request.setParam4("0");
        receptionistForPredicate.publish(request);
        */

        try {
          /*
          Thread.sleep(2000);
          MonitorServiceRequest request2 = requestor.newMessage();
          request2.setStatus(99);
          request2.setManager("ContextManager");
          requestor.publish(request2);
          */
          Thread.sleep(3000);

        
          
         // String message="";
          MonitorServiceRequest request = requestor.newMessage();
          //message = queryInput.nextLine();
          //setMessage(request,message);

          //test!!!!!!
          // request.setPredicate("currentObjectPerception");
          // request.setParam1("Object");
          // request.setParam2("Perception");
          // request.setParam3("0");
          // request.setParam4("0");
          // request.setStatus(100);
          // request.setManager("ContextManager");
          // requestor.publish(request);
          // Thread.sleep(5000);
         
         //start here!!!
          request.setPredicate("on_Physical");
          request.setParam1("Top");
          request.setParam2("Bottom");
          request.setParam3("0");
          request.setParam4("0");
          request.setStatus(100);
          request.setManager("TaskManager");
          requestor.publish(request);
          connectedNode.getLog().info(String.format("send ========> on_Physical Top Bottom 0 0 100"));
          Thread.sleep(5000);



          request.setPredicate("empty_hand");
          request.setParam1("Hand");
          request.setParam2("0");
          request.setParam3("0");
          request.setParam4("0");
          request.setStatus(100);
          request.setManager("TaskManager");
          requestor.publish(request);
          connectedNode.getLog().info(String.format("send ========> empty_hand Hand 0 0 0 100"));
          Thread.sleep(5000);


          request.setPredicate("opened_hand");
          request.setParam1("Hand");
          request.setParam2("0");
          request.setParam3("0");
          request.setParam4("0");
          request.setStatus(100);
          request.setManager("TaskManager");
          requestor.publish(request);
          connectedNode.getLog().info(String.format("send ========> opened_hand Hand 0 0 0 100"));
          Thread.sleep(5000);

          request.setPredicate("graspedBy");
          request.setParam1("Object");
          request.setParam2("Hand");
          request.setParam3("0");
          request.setParam4("0");
          request.setStatus(100);
          request.setManager("TaskManager");
          requestor.publish(request);
          connectedNode.getLog().info(String.format("send ========> graspedBy Object Hand 0 0 100"));
          Thread.sleep(5000);


          request.setPredicate("detected_object");
          request.setParam1("Object");
          request.setParam2("0");
          request.setParam3("0");
          request.setParam4("0");
          request.setStatus(100);
          request.setManager("TaskManager");
          requestor.publish(request);
          connectedNode.getLog().info(String.format("send ========> detected_object Object 0 0 0 100"));
          Thread.sleep(5000);


          request.setPredicate("locatedAt");
          request.setParam1("Object");
          request.setParam2("Location");
          request.setParam3("0");
          request.setParam4("0");
          request.setStatus(100);
          request.setManager("TaskManager");
          requestor.publish(request);
          connectedNode.getLog().info(String.format("send ========> locatedAt Object Location 0 0 100"));
          Thread.sleep(5000);

          request.setPredicate("in_ContGeneric");
          request.setParam1("InnerObj");
          request.setParam2("OuterObj");
          request.setParam3("0");
          request.setParam4("0");
          request.setStatus(100);
          request.setManager("TaskManager");
          requestor.publish(request);
          connectedNode.getLog().info(String.format("send ========> in_ContGeneric InnerObj OuterObj 0 0 100"));
          Thread.sleep(5000);

          request.setPredicate("aboveOf"); 
          request.setParam1("Top");
          request.setParam2("Bottom");
          request.setParam3("0");
          request.setParam4("0");
          request.setStatus(100);
          request.setManager("TaskManager");
          requestor.publish(request);
          connectedNode.getLog().info(String.format("send ========> aboveOf Top Bottom 0 0 100"));
          Thread.sleep(15000);

          request.setPredicate("belowOf");  
          request.setParam1("Bottom");
          request.setParam2("Top");
          request.setParam3("0");
          request.setParam4("0");
          request.setStatus(100);
          request.setManager("TaskManager");
          requestor.publish(request);
          connectedNode.getLog().info(String.format("send ========> belowOf Bottom Top 0 0 100"));
          Thread.sleep(15000);
          
          request.setPredicate("inFrontOf");
          request.setParam1("Front");
          request.setParam2("Back");
          request.setParam3("0");
          request.setParam4("0");
          request.setStatus(100);
          request.setManager("TaskManager");
          requestor.publish(request);
          connectedNode.getLog().info(String.format("send ========> inFrontOf Front Back 0 0 100"));
          Thread.sleep(15000);

          request.setPredicate("behindOf");
          request.setParam1("Back");
          request.setParam2("Front");
          request.setParam3("0");
          request.setParam4("0");
          request.setStatus(100);
          request.setManager("TaskManager");
          requestor.publish(request);
          connectedNode.getLog().info(String.format("send ========> behindOf Back Front 0 0 100"));
          Thread.sleep(15000);

          request.setPredicate("near");
          request.setParam1("Robot");
          request.setParam2("Object");
          request.setParam3("0");
          request.setParam4("0");
          request.setStatus(100);
          request.setManager("TaskManager");
          requestor.publish(request);
          connectedNode.getLog().info(String.format("send ========> near Robot Object 0 0 100"));
          Thread.sleep(5000);
          /*
          request.setPredicate("empty_container");
          request.setParam1("0");
          request.setParam2("0");
          request.setParam3("0");
          request.setParam4("0");
          request.setStatus(100);
          request.setManager("ContextManager");
          requestor.publish(request);
          Thread.sleep(5000);
          */
          /*
          while(true){
            //Thread.sleep(2000);
            //ContextManager.setMessage(request,scan.nextLine());
            //request.setManager("TaskManager");
    
            String message="";
            MonitorServiceRequest request = requestor.newMessage();
            message = queryInput.nextLine();
            setMessage(request,message);
            requestor.publish(request);
          }*/

        } catch (Exception e) {
          e.printStackTrace();
        }
      }

      public void setMessage(MonitorServiceRequest request, String query){
        String[] params = query.split(" ");
          request.setPredicate(params[0]);
          request.setParam1(params[1]);
          request.setParam2(params[2]);
          request.setParam3(params[3]);
          request.setParam4(params[4]);
          request.setStatus(Integer.parseInt(params[5]));//100);
          request.setManager("TaskManager");
      }
    }
