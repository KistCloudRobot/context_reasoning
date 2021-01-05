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

import rosjava_custom_srv.*;
import java.util.*;
import org.jpl7.*;
   
public class Parser {
    
     
     /*String rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#";
     String owl="http://www.w3.org/2002/07/owl#";
     String xml="http://www.w3.org/XML/1998/namespace";
     String owl2xml="http://www.w3.org/2006/12/owl2-xml#";
     String xsd="http://www.w3.org/2001/XMLSchema#";
     String knowrob="http://knowrob.org/kb/knowrob.owl#";
     String rdfs="http://www.w3.org/2000/01/rdf-schema#";
     String computable="http://knowrob.org/kb/computable.owl#";
     String owl2="http://www.w3.org/2006/12/owl2#";
     String arbi="http://www.arbi.com/ontologies/arbi.owl#";
     String protege="http://protege.stanford.edu/plugins/owl/protege#";
	 String srdl2_comp="http://knowrob.org/kb/srdl2-comp.owl#";
     */
     static String tempPrefix[][]={
      {"http://www.w3.org/1999/02/22-rdf-syntax-ns#","rdf"},
      {"http://www.w3.org/2002/07/owl#","owl"},
      {"http://www.w3.org/XML/1998/namespace","xml"},
      {"http://www.w3.org/2006/12/owl2-xml#","owl2xml"},
      {"http://www.w3.org/2001/XMLSchema#","xsd"},
      {"http://knowrob.org/kb/knowrob.owl#","knowrob"},
      {"http://www.w3.org/2000/01/rdf-schema#","rdfs"},
      {"http://knowrob.org/kb/computable.owl#","computable"},
      {"http://www.w3.org/2006/12/owl2#","owl2"},
      {"http://www.arbi.com/ontologies/arbi.owl#","arbi"},
      {"http://protege.stanford.edu/plugins/owl/protege#","protege"},
	  {"http://knowrob.org/kb/srdl2-comp.owl#","srdl2_comp"}
     };
    public static String transPredicateQuery(MonitorServiceRequest request){
        String predicateQuery = request.getPredicate() + "(";
    
        if(!request.getParam1().equals("0")){
            if(request.getParam1().contains("#"))
                predicateQuery += "'"+request.getParam1()+"'";
            else
                predicateQuery += request.getParam1();
        }
        if(!request.getParam2().equals("0")){
            if(request.getParam2().contains("#"))
                predicateQuery += ",'"+request.getParam2()+"'";
            else
                predicateQuery += ","+request.getParam2();
        }
        if(!request.getParam3().equals("0")){
            if(request.getParam3().contains("#"))
                predicateQuery += ",'"+request.getParam3()+"'";
            else
                predicateQuery += ","+request.getParam3();
        }
        if(!request.getParam4().equals("0")){
            if(request.getParam4().contains("#"))
                predicateQuery += ",'"+request.getParam4()+"'";
            else
                predicateQuery += ","+request.getParam4();
        }
        predicateQuery +=").";
            return predicateQuery;
    }

    public static String transPredicateQuery(MainServiceRequest request){
        String predicateQuery = request.getPredicate() + "(";
    
        if(!request.getParam1().equals("0")){
            if(request.getParam1().contains("#"))
                predicateQuery += "'"+request.getParam1()+"'";
            else
                predicateQuery += request.getParam1();
        }
        if(!request.getParam2().equals("0")){
            if(request.getParam2().contains("#"))
                predicateQuery += ",'"+request.getParam2()+"'";
            else
                predicateQuery += ","+request.getParam2();
        }
        if(!request.getParam3().equals("0")){
            if(request.getParam3().contains("#"))
                predicateQuery += ",'"+request.getParam3()+"'";
            else
                predicateQuery += ","+request.getParam3();
        }
        if(!request.getParam4().equals("0")){
            if(request.getParam4().contains("#"))
                predicateQuery += ",'"+request.getParam4()+"'";
            else
                predicateQuery += ","+request.getParam4();
        }
        predicateQuery +=").";
            return predicateQuery;
    }

    public static String text_transPredicate(MonitorServiceResponse response){
        String transPredicate = response.getPredicate() + "(";
    
        if(!response.getParam1().equals("0")){
            if(response.getParam1().contains("#"))
                transPredicate += response.getParam1();
            else
                transPredicate += response.getParam1();
        }
        if(!response.getParam2().isEmpty()){
            if(response.getParam2().contains("#"))
                transPredicate += ","+response.getParam2();
            else
                transPredicate += ","+response.getParam2();
        }
        if(!response.getParam3().isEmpty()){
            if(response.getParam3().contains("#"))
                transPredicate += ","+response.getParam3();
            else
                transPredicate += ","+response.getParam3();
        }
        if(!response.getParam4().isEmpty()){
            if(response.getParam4().contains("#"))
                transPredicate += ","+response.getParam4();
            else
                transPredicate += ","+response.getParam4();
        }
            transPredicate +=").";
            return transPredicate;
    }

    public static String transPredicateResponse(MonitorServiceRequest request, Map<String, Term>[] queryResponse){
        String predicateResponse = "";
        String responseString;

        for(int i = 0; i < queryResponse.length; i++){
            responseString = request.getPredicate() + "(";

            if(!request.getParam1().equals("0")){
                if(request.getParam1().contains("#"))
                    responseString += "'"+request.getParam1()+"'";
                else if(request.getParam1().contains("[|]")){
                    String convertPose = request.getParam1().replace("'[|]'","").replace("(","").replace(")","");
                    responseString += "["+convertPose+"]";
                }
                else
                    responseString += queryResponse[i].get(request.getParam1());
                
                    
            }
            if(!request.getParam2().equals("0")){
                if(request.getParam2().contains("#"))
                    responseString += ",'"+request.getParam2()+"'";
                else if(request.getParam2().contains("[|]")){
                    String convertPose = request.getParam2().replace("[|]","").replace("(","").replace(")","");
                    responseString += "["+convertPose+"]";
                }
                else
                    responseString += ","+queryResponse[i].get(request.getParam2());
            }
            if(!request.getParam3().equals("0")){
                if(request.getParam3().contains("#"))
                    responseString += ",'"+request.getParam3()+"'";
                else if(request.getParam3().contains("[|]")){
                    String convertPose = request.getParam3().replace("[|]","").replace("(","").replace(")","");
                    responseString += "["+convertPose+"]";
                }
                else
                    responseString += ","+queryResponse[i].get(request.getParam3());
            }
            if(!request.getParam4().equals("0")){
                if(request.getParam4().contains("#"))
                    responseString += ",'"+request.getParam4()+"'";
                else if(request.getParam4().contains("[|]")){
                    String convertPose = request.getParam4().replace("[|]","").replace("(","").replace(")","");
                    responseString += "["+convertPose+"]";
                }
                else
                    responseString += ","+queryResponse[i].get(request.getParam4());
            }
            responseString +=").";
            predicateResponse += responseString;
            responseString = "";
        }
        return predicateResponse;
    }

    public static MonitorServiceResponse setResponse(MonitorServiceRequest request, Map<String, Term> queryResponse,MonitorServiceResponse response){
        response.setPredicate(request.getPredicate());
        
        if(!request.getParam1().equals("0")){
            if(request.getParam1().contains("#"))
                response.setParam1(request.getParam1());
            else
                response.setParam1(queryResponse.get(request.getParam1()).toString());
        }
        if(!request.getParam2().equals("0")){
            if(request.getParam2().contains("#"))
                response.setParam2(request.getParam2());
            else
                response.setParam2(queryResponse.get(request.getParam2()).toString());
        }
        if(!request.getParam3().equals("0")){
            if(request.getParam3().contains("#"))
                response.setParam3(request.getParam3());
            else
                response.setParam3(queryResponse.get(request.getParam3()).toString());
        }
        if(!request.getParam4().equals("0")){
            if(request.getParam4().contains("#"))
                response.setParam4(request.getParam4());
            else
                response.setParam4(queryResponse.get(request.getParam4()).toString());
        }

        //ContextManager.log.info("set Response"+ response.getPredicate()+","+response.getParam1()+","+response.getParam2());
        return response;
        
    }

    public static String transPredicateQuery(TempQuery request){
        String predicateQuery = request.getPredicate() + "(";
    
        if(!request.getParam1().equals("0")){
            if(request.getParam1().contains("#"))
                predicateQuery += "'"+request.getParam1()+"'";
            else
                predicateQuery += request.getParam1();
        }
        if(!request.getParam2().equals("0")){
            if(request.getParam2().contains("#"))
                predicateQuery += ",'"+request.getParam2()+"'";
            else
                predicateQuery += ","+request.getParam2();
        }
        if(!request.getParam3().equals("0")){
            if(request.getParam3().contains("#"))
                predicateQuery += ",'"+request.getParam3()+"'";
            else
                predicateQuery += ","+request.getParam3();
        }
        if(!request.getParam4().equals("0")){
            if(request.getParam4().contains("#"))
                predicateQuery += ",'"+request.getParam4()+"'";
            else
                predicateQuery += ","+request.getParam4();
        }
        predicateQuery +=").";
            return predicateQuery;
    }

    public static String text_transPredicate(TempQuery response){
        String transPredicate = response.getPredicate() + "(";
    
        if(!response.getParam1().equals("0")){
            if(response.getParam1().contains("#"))
                transPredicate += response.getParam1();
            else
                transPredicate += response.getParam1();
        }
        if(!response.getParam2().isEmpty()){
            if(response.getParam2().contains("#"))
                transPredicate += ","+response.getParam2();
            else
                transPredicate += ","+response.getParam2();
        }
        if(!response.getParam3().isEmpty()){
            if(response.getParam3().contains("#"))
                transPredicate += ","+response.getParam3();
            else
                transPredicate += ","+response.getParam3();
        }
        if(!response.getParam4().isEmpty()){
            if(response.getParam4().contains("#"))
                transPredicate += ","+response.getParam4();
            else
                transPredicate += ","+response.getParam4();
        }
            transPredicate +=").";
            return transPredicate;
    }

    public static String transPredicateResponse(TempQuery request, Map<String, Term>[] queryResponse){
        String predicateResponse = "";
        String responseString;

        for(int i = 0; i < queryResponse.length; i++){
            responseString = request.getPredicate() + "(";

            if(!request.getParam1().equals("0")){
                if(request.getParam1().contains("#"))
                    responseString += "'"+request.getParam1()+"'";
                else
                    responseString += queryResponse[i].get(request.getParam1());
            }
            if(!request.getParam2().equals("0")){
                if(request.getParam2().contains("#"))
                    responseString += ",'"+request.getParam2()+"'";
                else
                    responseString += ","+queryResponse[i].get(request.getParam2());
            }
            if(!request.getParam3().equals("0")){
                if(request.getParam3().contains("#"))
                    responseString += ",'"+request.getParam3()+"'";
                else
                    responseString += ","+queryResponse[i].get(request.getParam3());
            }
            if(!request.getParam4().equals("0")){
                if(request.getParam4().contains("#"))
                    responseString += ",'"+request.getParam4()+"'";
                else
                    responseString += ","+queryResponse[i].get(request.getParam4());
            }
            responseString +=").";
            predicateResponse += responseString;
            responseString = "";
        }
        return predicateResponse;
    }

    public static MonitorServiceResponse setResponse(TempQuery request, Map<String, Term> queryResponse,MonitorServiceResponse response){
        response.setPredicate(request.getPredicate());
        
        if(!request.getParam1().equals("0")){
            if(request.getParam1().contains("#"))
                response.setParam1(request.getParam1());
            else
                response.setParam1(queryResponse.get(request.getParam1()).toString());
        }
        if(!request.getParam2().equals("0")){
            if(request.getParam2().contains("#"))
                response.setParam2(request.getParam2());
            else
                response.setParam2(queryResponse.get(request.getParam2()).toString());
        }
        if(!request.getParam3().equals("0")){
            if(request.getParam3().contains("#"))
                response.setParam3(request.getParam3());
            else
                response.setParam3(queryResponse.get(request.getParam3()).toString());
        }
        if(!request.getParam4().equals("0")){
            if(request.getParam4().contains("#"))
                response.setParam4(request.getParam4());
            else
                response.setParam4(queryResponse.get(request.getParam4()).toString());
        }

        //ContextManager.log.info("set Response"+ response.getPredicate()+","+response.getParam1()+","+response.getParam2());
        return response;
        
    }

    public static String longNameToShortName(String name){
        if(name==null)return name;
        for(int i=0;i<tempPrefix.length;i++){
            if(name.contains(tempPrefix[i][0])){
                name=name.replace(tempPrefix[i][0],tempPrefix[i][1]+":");
                break;
                }
            }
        return name;


    }
}
