package org.ros.rosjava_context_manager;

import javafx.application.Application;
import javafx.stage.Stage;
import javafx.scene.Scene;
import javafx.scene.layout.AnchorPane;
import javafx.fxml.FXMLLoader;


public class FSDUI extends Application {
	private FXMLLoader loader;
	private static MKMController MKMC;
	private FSD fsd;

	public static FSDUI getInstance(){
		return new FSDUI();
	}
	
	@Override
	public void start(Stage primaryStage) {
		try {
			
			loader = new FXMLLoader();
			loader.setLocation(getClass().getResource("/FSDLayout.fxml"));
			
			AnchorPane root = (AnchorPane)loader.load();
			Scene scene = new Scene(root);
			scene.getStylesheets().add(getClass().getResource("/application.css").toExternalForm());
			primaryStage.setScene(scene);		
			primaryStage.setTitle("ContextManager");
			MKMC = loader.getController();
			primaryStage.show();
		} catch(Exception e) {
			e.printStackTrace();
		}
	}
	
	public MKMController getMKMController() {
		return MKMC;
	}

	/*public void updateFSD(String fsd){
		this.fsd = new FSD();
		fsd.setPredicate();
		fsd.setParam1();
		fsd.setParam2();
		fsd.setParam3();
		fsd.setParam4();
		MKMC.updateFSD(fsd);
	}*/

	public void clearFSD(){
		MKMC.clearFSD();
	}
	public void updateFSD(FSD fsd){
		MKMC.updateFSD(fsd);
	}

	public static void main(String[] args) {
		 launch(args);
	}
}
