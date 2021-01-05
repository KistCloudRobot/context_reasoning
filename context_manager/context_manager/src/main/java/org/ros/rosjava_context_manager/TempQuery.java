package org.ros.rosjava_context_manager;

public class TempQuery{
     String predicate;
     String param1;
     String param2;
     String param3;
     String param4;
     String manager;

     public void setPredicate(String predicate){this.predicate=predicate;}
     public void setParam1(String param1){this.param1=param1;}
     public void setParam2(String param2){this.param2=param2;}
     public void setParam3(String param3){this.param3=param3;}
     public void setParam4(String param4){this.param4=param4;}
     public void setManager(String manager){this.manager=manager;}

     public String getPredicate(){return predicate;}
     public String getParam1(){return param1;}
     public String getParam2(){return param2;}
     public String getParam3(){return param3;}
     public String getParam4(){return param4;}
     public String getManager(){return manager;}

     public String toString(){
         return "Query Elements : " + predicate + " "+ param1 +  " "+ param2 + " "+  param3+ " "+  param4 + " "+manager;
     }
   }