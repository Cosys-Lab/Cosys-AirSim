from __future__ import print_function
from PySide2 import QtWidgets, QtCore, QtGui
import MaxPlus
import os
import pymxs
import math
import subprocess

###################################################################
#   3dsMax Unreal 4 Instance Exporter by Wouter Jansen            #
#                                                                 #
#   When you have many duplicate objects (instances) and want to  # 
#   setup their location in 3dsMax and have those locations in    #
#   your Unreal 4 level you can use this script.                  #
#                                                                 #
#   Steps:                                                        #
#   1. Select your objects in 3dsMax (v2020 was used)             #
#   2. Run this script                                            #
#   3. Set the folder where to save the instances to              #
#   4. Press the export button                                    #
#   5. Open the instances.txt that was created, copy it's content #
#      to your clipboard	                     	              #
#   6. Open your Unreal 4 editor (v4.22 was used)                 #
#   7. In the viewport, paste your clipboard.                     #
#   8. Observe all the cubes that were spawned. Based on the      #
#      actor names you now can replace these with the             #
#      actual meshes you want for each instance type.             #
#   9. Be annoyed that Unreal doesn't support FBX instances yet.  #
#                                                                 #
#   Heavily inspired and based on the TS Tools by Tom Shannon.    #
#   http://www.tomshannon3d.com/2014/09/tstoolsv11.html           #
###################################################################

class PySideTextField(QtWidgets.QLineEdit):
    def focusInEvent(self, event):
        MaxPlus.CUI.DisableAccelerators()

    def focusOutEvent(self, event):
        MaxPlus.CUI.EnableAccelerators()
		

class PySideUI(QtWidgets.QDialog):


 def __init__(self, parent=MaxPlus.GetQMaxMainWindow()):
     super(PySideUI, self).__init__(parent)
     self.setWindowTitle("Unreal 4 - Object Instancing Export")	 
     self.initUI()
	 
	 
 def execute_export(self):
	self.executeInfoLabel.setVisible(0)
	
	mySel = MaxPlus.SelectionManager.Nodes
	print("Starting...")
	
	try:		
		
		instancesFile = open(self.folderTextField.text().replace("\\", "/") + "\instances.txt", "w")
		
		objectCount = sum(1 for x in mySel)
		print("Object Count:",objectCount)
		
		if objectCount is not 0:
			print("Begin Map\n   Begin Level", file = instancesFile)
			self.executeInfoLabel.setText("Exporting " + str(objectCount) + " objects...")
			self.executeInfoLabel.setVisible(1)
			
			mySel = MaxPlus.SelectionManager.Nodes
			index = 0
			for each in mySel:
				index = index + 1
				print("Adding object", each.Name)
				
				scaledMatrix = each.Transform
				setToScale = MaxPlus.Point3(1,1,1) / scaledMatrix.Scale
				scaledMatrix.PreScale(setToScale)
				
				print("      Begin Actor Class=StaticMeshActor Name={} Archetype=StaticMeshActor'/Script/Engine.Default__StaticMeshActor'".format(each.Name.encode("utf-8")), file = instancesFile)
				print("         Begin Object Class=StaticMeshComponent Name=StaticMeshComponent0 ObjName=StaticMeshComponent0 Archetype=StaticMeshComponent'/Script/Engine.Default__StaticMeshActor:StaticMeshComponent0'\n         End Object", file = instancesFile)
				print("         Begin Object Name=StaticMeshComponent0\n            StaticMesh=StaticMesh'/Engine/EditorMeshes/EditorCube.EditorCube'", file = instancesFile)
				print("            RelativeLocation=(X={},Y={},Z={})".format(round(each.Position.GetX(), 2) + 0, round(each.Position.GetY() * -1, 2) + 0, round(each.Position.GetZ(), 2) + 0), file = instancesFile)
				print("            RelativeScale3D=(X=1.0,Y=1.0,Z=1.0)", file = instancesFile)
				print("            RelativeRotation=(Pitch={},Yaw={},Roll={})".format(round(scaledMatrix.Rotation.GetEuler().GetY() * -1 * 180 / math.pi , 4) + 0, round(scaledMatrix.Rotation.GetEuler().GetZ() * -1 * 180 / math.pi , 4) + 0, round(scaledMatrix.Rotation.GetEuler().GetX() * 180 /  math.pi , 4) + 0), file = instancesFile)
				print("            CustomProperties \n         End Object\n         StaticMeshComponent=StaticMeshComponent0\n         Components(0)=StaticMeshComponent0\n         RootComponent=StaticMeshComponent0", file = instancesFile)
				print("         ActorLabel=\"{}\"".format(each.Name.encode("utf-8")), file = instancesFile)
				print("      End Actor", file = instancesFile)				
				print("Added!")
				
			print("   End Level\nBegin Surface\nEnd Surface\nEnd Map", file = instancesFile)
			print("Export completed!")
			self.executeInfoLabel.setText("Export completed for " + str(objectCount) + " objects!\nYou can now open the instances.txt file,\ncopy all the text to the clipboard and paste it in an Unreal 4 editor to spawn cubes on the object locations.\nWhich you can then replace with the instanced models that are imported separately.")
			subprocess.Popen("explorer /select," + self.folderTextField.text()+ "\instances.txt")
			
		else:
			print("Nothing Selected!")
			self.executeInfoLabel.setVisible(1)
			self.executeInfoLabel.setText("No objects selected!")

	except Exception, e:
		print("Error: ",e)
		self.executeInfoLabel.setText(str(e))
		self.executeInfoLabel.setVisible(1)
	 

 def initUI(self):
     self.mainLayout = QtWidgets.QVBoxLayout()

     infoLabel = QtWidgets.QLabel("Choose folder to save instances.txt:                                                 ")
     self.mainLayout.addWidget(infoLabel)
	 
     self.folderTextField = PySideTextField()
     maxExportDir = MaxPlus.PathManager.GetExportDir()
     self.folderTextField.setText(maxExportDir)
     self.mainLayout.addWidget(self.folderTextField)		 

     exportBtn = QtWidgets.QPushButton("Export")
     self.mainLayout.addWidget(exportBtn)	 
     exportBtn.clicked.connect(self.execute_export)
		  
     self.executeInfoLabel = QtWidgets.QLabel("You shouldn't see this yet.")
     self.mainLayout.addWidget( self.executeInfoLabel)
     self.executeInfoLabel.setVisible(0) 

     self.setLayout(self.mainLayout)


if __name__ == "__main__":

 try:
    ui.close()
 except:
    pass

 ui = PySideUI()
 ui.show()