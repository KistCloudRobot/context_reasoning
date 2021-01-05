
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
    public class ClientService extends AbstractNodeMain {

      static ServiceClient<MonitorSimilarServiceRequest, MonitorSimilarServiceResponse> serviceClient;

      static Log log;

      MonitorSimilarServiceRequest request;

      @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("rosjava_tutorial_custom_custom_services/client_service");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    try {
      log = connectedNode.getLog();

      serviceClient = connectedNode.newServiceClient("context_manager/monitor/service", MonitorSimilarService._TYPE);
      request = serviceClient.newMessage();
    } catch (ServiceNotFoundException e) {
      throw new RosRuntimeException(e);
    }


    while(true){
      sendMessage("objectSize http://www.arbi.com/ontologies/arbi.owl#milk P 0 0 100");
      sleep(50);
      sendMessage("on_Physical H P 0 0 100");
       sleep(50);
       sendMessage("currentObjectPerception H P 0 0 100");
       sleep(50);
       sendMessage("currentHandPose H P 0 0 100");
       sleep(50);
       sendMessage("empty_hand H 0 0 0 100");
      sleep(50);
       sendMessage("closed_hand H 0 0 0 100");
      sleep(50);
       sendMessage("opened_hand H 0 0 0 100");
      sleep(50);
       sendMessage("full_hand H 0 0 0 100");
      sleep(50);
       sendMessage("empty_container H 0 0 0 100");
      sleep(50);

      sendMessage("objectSize http://www.arbi.com/ontologies/arbi.owl#milk P 0 0 100");
      sleep(50);
      sendMessage("on_Physical H P 0 0 100");
       sleep(50);
       sendMessage("currentObjectPerception H P 0 0 100");
       sleep(50);
       sendMessage("currentHandPose H P 0 0 100");
       sleep(50);
       sendMessage("empty_hand H 0 0 0 100");
      sleep(50);
       sendMessage("closed_hand H 0 0 0 100");
      sleep(50);
       sendMessage("opened_hand H 0 0 0 100");
      sleep(50);
       sendMessage("full_hand H 0 0 0 100");
      sleep(50);
       sendMessage("empty_container H 0 0 0 100");
      sleep(50);

      sendMessage("objectSize http://www.arbi.com/ontologies/arbi.owl#milk P 0 0 100");
      sleep(50);
      sendMessage("on_Physical H P 0 0 100");
       sleep(50);
       sendMessage("currentObjectPerception H P 0 0 100");
       sleep(50);
       sendMessage("currentHandPose H P 0 0 100");
       sleep(50);
       sendMessage("empty_hand H 0 0 0 100");
      sleep(50);
       sendMessage("closed_hand H 0 0 0 100");
      sleep(50);
       sendMessage("opened_hand H 0 0 0 100");
      sleep(50);
       sendMessage("full_hand H 0 0 0 100");
      sleep(50);
       sendMessage("empty_container H 0 0 0 100");
      sleep(50);


      // sleep(1000);

    }
    
    //set the request/size

    
  }

  public void sendMessage(String str){
setMessage(request,str);

    serviceClient.call(request, new ServiceResponseListener<MonitorSimilarServiceResponse>() {
      @Override
      public void onSuccess(MonitorSimilarServiceResponse response) {
      if (response.getResponse().size()!=0){
        log.info("Service successed");
        for(int i=0;i<response.getResponse().size();i++)
          System.out.println(response.getResponse().get(i).getPredicate() + " " +response.getResponse().get(i).getParam1()+ " " +response.getResponse().get(i).getParam2()+ " " +response.getResponse().get(i).getParam3()+ " " +response.getResponse().get(i).getParam4());
        
      }
      }

      @Override
      public void onFailure(RemoteException e) {
        log.info("Service failed");
        throw new RosRuntimeException(e);
      }
    });
  }

      public void setMessage(MonitorSimilarServiceRequest request, String query){
        String[] params = query.split(" ");
          request.setPredicate(params[0]);
          request.setParam1(params[1]);
          request.setParam2(params[2]);
          request.setParam3(params[3]);
          request.setParam4(params[4]);
          request.setStatus(Integer.parseInt(params[5]));//100);
          request.setManager("TaskManager");
      }


	public void sleep(int n){
		try{
			Thread.sleep(n);
		}catch(Exception e){}
	}
    }
