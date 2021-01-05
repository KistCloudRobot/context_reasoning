package org.ros.rosjava_context_manager;


import java.awt.*;
import java.awt.event.*;

import javax.swing.*;
import javax.swing.text.*;


public class FSDUI3 extends JFrame {
 
 //Panel, button 및 JtextArea
 JPanel panel;
 JScrollPane sp;
 JTextArea fsdTextArea;
 String fsd="";

 
 /*****************************************************************************
  * 생성자  
  ****************************************************************************/
 public FSDUI3(){
  
  setLayout(null); //Layout을 NULL로 설정 (컴포넌트의 위치를 사용자가 설정해주어야 함)  
  CreatePanel();
  setDefaultListener();
  setTitle("FSD"); //Frame의 타이틀 이름 주기 
  setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE); //Frame의 X를 누를경우 종료 
  setSize(400, 500); //Frame의 크기 설정 
  setVisible(true); //생성한 Frame을 윈도우에 뿌리기 
 }
 

 /*****************************************************************************
  * Panel을 생성 
  ****************************************************************************/
 private void CreatePanel() {
  
  panel = new JPanel(); //패널을 생성 
  panel.setLayout(new BorderLayout()); //패널의 Layout을 NULL
  panel.setBounds(0, 0, 400, 500); //패널의 크기 및 위치 지정 (x,y로 부터 넓이(width, 높이(height))
  
   fsdTextArea = new JTextArea();  //JTextArea 생성 
   fsdTextArea.setBounds(10, 10, 380, 480); //JTeatArea 크기 및 위치 지정 
   fsdTextArea.setEditable(false); //실행시 JtextArea edit 금지 (글을 쓸 수 없음) true면 가능
   
   sp=new JScrollPane(fsdTextArea);

   DefaultCaret caret = (DefaultCaret) fsdTextArea.getCaret();
   caret.setUpdatePolicy(DefaultCaret.ALWAYS_UPDATE);
  
   panel.add(sp, BorderLayout.CENTER); //패널에 Textarea add
  
  setLayout(new BorderLayout());
  add(panel);  //Frame에 Panel add (JFrame을 상속받았기에 this(생략가능)가 JFrame)
  
 }

 private void setDefaultListener() {
 panel.addComponentListener(new ComponentAdapter() {
     @Override
     public void componentResized(ComponentEvent e) {
    	// fsdTextArea.setBounds(10, 10, e.getComponent().getSize().width, e.getComponent().getSize().height); //JTeatArea 크기 및 위치 지정 
       //System.out.println("Resized to " + e.getComponent().getSize());
     }
     @Override
     public void componentMoved(ComponentEvent e) {
       //System.out.println("Moved to " + e.getComponent().getLocation());
     }
   });
 }
 
 public void updateFSD(String fsd) {
	 this.fsd+=fsd+"\n";
	 fsdTextArea.setText(this.fsd);
 }
}