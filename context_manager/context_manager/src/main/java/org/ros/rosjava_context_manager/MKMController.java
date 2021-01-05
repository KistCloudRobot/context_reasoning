package org.ros.rosjava_context_manager;

import java.io.IOException;

import com.sun.javafx.scene.control.skin.*;

import javafx.application.Platform;
import javafx.beans.property.ReadOnlyStringWrapper;
import javafx.beans.value.ObservableValue;
import javafx.collections.*;
import javafx.collections.transformation.FilteredList;
import javafx.fxml.FXML;
import javafx.scene.control.*;
import javafx.scene.control.TableColumn.CellDataFeatures;
import javafx.util.Callback;


import java.time.format.DateTimeFormatter;
import java.time.LocalDateTime; 

public class MKMController {
	/*@FXML
	private ListView<String> fsdListView;
	
	@FXML
	private TextArea fsdTextArea;
	
	private FilteredList<String> fsdFilteredList;
	private ObservableList<String> fsdList;
	
	private String textArea="";
	*/
	
	@FXML
	private TableView<FSD> fsdTableView;
	
	@FXML
	private TableColumn<FSD,String> fsdPredicateColumn;
	
	@FXML
	private TableColumn<FSD,String>  fsdVar1Column;
	
	@FXML
	private TableColumn<FSD,String>  fsdVar2Column;
	
	@FXML
	private TableColumn<FSD,String>  fsdVar3Column;
	
	@FXML
	private TableColumn<FSD,String>  fsdVar4Column;
	
	@FXML
	private Label fsdLastTimeLabel;

	private FilteredList<FSD> fsdFilteredList;
	private ObservableList<FSD> fsdList;

	static DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss");
	
	
	@FXML
	private void initialize() throws IOException {
		/*fsdList= FXCollections.observableArrayList();
		fsdListView.setItems(fsdList);
		fsdListView.setVisible(true);*/
		
		//fsdList= FXCollections.observableArrayList();
		//fsdTextArea.setText(fsdList);
		//fsdTextArea.setVisible(true);
		fsdList= FXCollections.observableArrayList();
		fsdTableView.setItems(fsdList);
		/*fsdPredicateColumn.setCellValueFactory(data -> data.getValue().getPredicate());
		fsdVar1Column.setCellValueFactory(data -> data.getValue().getParam1());
		fsdVar2Column.setCellValueFactory(data -> data.getValue().getParam2());
		fsdVar3Column.setCellValueFactory(data -> data.getValue().getParam3());
		fsdVar4Column.setCellValueFactory(data -> data.getValue().getParam4());

		
		
		fsdTableView.skinProperty().addListener((obs, oldSkin, newSkin) -> {
		    final TableHeaderRow header = (TableHeaderRow) fsdTableView.lookup("TableHeaderRow");
		    header.reorderingProperty().addListener((o, oldVal, newVal) -> header.setReordering(false));
		});
*/
		fsdPredicateColumn.setCellValueFactory(new Callback<CellDataFeatures<FSD, String>, ObservableValue<String>>() {
    		@Override 
			public ObservableValue<String> call(CellDataFeatures<FSD, String> p) {
        		return new ReadOnlyStringWrapper(Parser.longNameToShortName(p.getValue().getPredicate()));
    		}
		});
		fsdVar1Column.setCellValueFactory(new Callback<CellDataFeatures<FSD, String>, ObservableValue<String>>() {
    		@Override 
			public ObservableValue<String> call(CellDataFeatures<FSD, String> p) {
        		return new ReadOnlyStringWrapper(Parser.longNameToShortName(p.getValue().getParam1()));
    		}
		});
		fsdVar2Column.setCellValueFactory(new Callback<CellDataFeatures<FSD, String>, ObservableValue<String>>() {
    		@Override 
			public ObservableValue<String> call(CellDataFeatures<FSD, String> p) {
        		return new ReadOnlyStringWrapper(Parser.longNameToShortName(p.getValue().getParam2()));
    		}
		});
		fsdVar3Column.setCellValueFactory(new Callback<CellDataFeatures<FSD, String>, ObservableValue<String>>() {
    		@Override 
			public ObservableValue<String> call(CellDataFeatures<FSD, String> p) {
        		return new ReadOnlyStringWrapper(Parser.longNameToShortName(p.getValue().getParam3()));
    		}
		});
		fsdVar4Column.setCellValueFactory(new Callback<CellDataFeatures<FSD, String>, ObservableValue<String>>() {
    		@Override 
			public ObservableValue<String> call(CellDataFeatures<FSD, String> p) {
        		return new ReadOnlyStringWrapper(Parser.longNameToShortName(p.getValue().getParam4()));
    		}
		});
	}

	public void clearFSD(){
		fsdList.clear();
		//fsdTableView.refresh();
	}
	public void updateFSD(String fsd) {
		//fsdList.add(fsd);
		//fsdListView.refresh();
		/*textArea+=fsd+"\n";
		fsdTextArea.setText(textArea);
		*/
		
	}
	
	public void updateFSD(FSD fsd) {
		// fsdList.clear();
		fsdList.add(fsd);
		Platform.runLater(new Runnable(){
			 @Override
			    public void run() {
   			fsdLastTimeLabel.setText("Last Time : " + dtf.format(LocalDateTime.now()));
			 }
		});
		fsdTableView.refresh();
	}

	
}
