package org.ros.rosjava_context_manager;

import javafx.beans.property.*;
public class FSD{
     StringProperty predicate;
     StringProperty param1;
     StringProperty param2;
     StringProperty param3;
     StringProperty param4;

     public FSD() {
    	predicate=new SimpleStringProperty();
    	param1=new SimpleStringProperty();
    	param2=new SimpleStringProperty();
    	param3=new SimpleStringProperty();
    	param4=new SimpleStringProperty();
     }
     public void setPredicate(String predicate){this.predicate.set(predicate);}
     public void setParam1(String param1){this.param1.set(param1);}
     public void setParam2(String param2){this.param2.set(param2);}
     public void setParam3(String param3){this.param3.set(param3);}
     public void setParam4(String param4){this.param4.set(param4);}

     public String getPredicate(){return predicate.get();}
     public String getParam1(){return param1.get();}
     public String getParam2(){return param2.get();}
     public String getParam3(){return param3.get();}
     public String getParam4(){return param4.get();}

     public StringProperty predicateProperty(){return predicate;}
     public StringProperty param1Property(){return param1;}
     public StringProperty param2Property(){return param2;}
     public StringProperty param3Property(){return param3;}
     public StringProperty param4Property(){return param4;}


     public String toStringProperty(){
         return "Query Elements : " + predicate + " "+ param1 +  " "+ param2 + " "+  param3+ " "+  param4;
     }
   }
