# Form implementation generated from reading ui file 'rosbag_converter.ui'
#
# Created by: PyQt6 UI code generator 6.6.0
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.

from pathlib import Path
from PyQt6 import QtCore, QtGui, QtWidgets
from converter import Converter
from DataIO import BagDataIO

class Ui_MainWindow(object):

    def __init__(self) -> None:
        self.converter = Converter()
        self.bag = BagDataIO()
        self.pwd = str(Path.cwd())

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(parent=MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayoutWidget = QtWidgets.QWidget(parent=self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(50, 190, 401, 341))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.buttonBox = QtWidgets.QDialogButtonBox(parent=self.centralwidget)
        self.buttonBox.setGeometry(QtCore.QRect(550, 500, 193, 28))
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.StandardButton.Cancel|QtWidgets.QDialogButtonBox.StandardButton.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.buttonBox.accepted.connect(self._onAccepted)
        self.buttonBox.rejected.connect(QtCore.QCoreApplication.instance().quit)
        self.description_1 = QtWidgets.QLabel(parent=self.centralwidget)
        self.description_1.setGeometry(QtCore.QRect(50, 10, 141, 21))
        self.description_1.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.NoTextInteraction)
        self.description_1.setObjectName("label_3")
        self.selected_bag = QtWidgets.QLabel(parent=self.centralwidget)
        self.selected_bag.setGeometry(QtCore.QRect(50, 30, 561, 31))
        self.selected_bag.setFrameShape(QtWidgets.QFrame.Shape.Box)
        self.selected_bag.setFrameShadow(QtWidgets.QFrame.Shadow.Plain)
        self.selected_bag.setObjectName("label")
        self.select_bag = QtWidgets.QPushButton(parent=self.centralwidget)
        self.select_bag.setGeometry(QtCore.QRect(650, 30, 93, 28))
        self.select_bag.setObjectName("pushButton")
        self.select_bag.clicked.connect(self._showOpenDialog)
        self.description_2 = QtWidgets.QLabel(parent=self.centralwidget)
        self.description_2.setGeometry(QtCore.QRect(50, 80, 201, 21))
        self.description_2.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.NoTextInteraction)
        self.description_2.setObjectName("label_4")
        self.selected_outdir = QtWidgets.QLabel(parent=self.centralwidget)
        self.selected_outdir.setGeometry(QtCore.QRect(50, 110, 561, 31))
        self.selected_outdir.setFrameShape(QtWidgets.QFrame.Shape.Box)
        self.selected_outdir.setText("")
        self.selected_outdir.setObjectName("label_2")
        self.select_output = QtWidgets.QPushButton(parent=self.centralwidget)
        self.select_output.setGeometry(QtCore.QRect(650, 110, 93, 28))
        self.select_output.setObjectName("select_output")
        self.select_output.clicked.connect(self._showOutputDialog)
        self.description_3 = QtWidgets.QLabel(parent=self.centralwidget)
        self.description_3.setGeometry(QtCore.QRect(50, 160, 211, 21))
        self.description_3.setObjectName("label_5")
        self.instruction = QtWidgets.QLabel(parent=self.centralwidget)
        self.instruction.setGeometry(QtCore.QRect(500, 210, 241, 241))
        self.instruction.setAlignment(QtCore.Qt.AlignmentFlag.AlignLeading|QtCore.Qt.AlignmentFlag.AlignLeft|QtCore.Qt.AlignmentFlag.AlignTop)
        self.instruction.setWordWrap(True)
        self.instruction.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.NoTextInteraction)
        self.instruction.setObjectName("label_6")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(parent=MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 25))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(parent=MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.description_1.setText(_translate("MainWindow", "Selected Input Bag File"))
        self.select_bag.setText(_translate("MainWindow", "Select bag file"))
        self.description_2.setText(_translate("MainWindow", "Selected Output Directory(Folder)"))
        self.select_output.setText(_translate("MainWindow", "Select out dir"))
        self.description_3.setText(_translate("MainWindow", "Available Topics in selected bag file"))
        self.instruction.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-size:11pt;\">1. Selected the target bag file.<br/></span></p><p><span style=\" font-size:11pt;\">2. Selected the convert result output root dir<br/></span></p><p><span style=\" font-size:11pt;\">3. Selected the topics which wolud convert.<br/></span></p><p><span style=\" font-size:11pt;\">4. Press OK</span></p></body></html>"))

    def _showOpenDialog(self):
        bag_path = QtWidgets.QFileDialog.getOpenFileName(None, 'select bag file', self.pwd, '*.bag')
        if bag_path[0]:
            self.selected_bag.setText(bag_path[0])
            self.openbagpd = QtWidgets.QProgressDialog('Processing', 'Cancel', 0, 0, None)
            self.bag.open(bag_path[0])
            self.openbagpd.close()
            self._set_avaliable_topics(self.bag.get_topics())

    def _showOutputDialog(self):
        out_dir = QtWidgets.QFileDialog.getExistingDirectory(None, 'Select out dir', self.pwd, options=QtWidgets.QFileDialog.Option.ShowDirsOnly)
        target_dir = 'bag_output'
        out_dir = Path('/').joinpath(out_dir, target_dir)
        self.selected_outdir.setText(out_dir.as_posix())
        self.bag.set_outdir_path(out_dir)
        self.outdir_ready = True
            

    def _set_avaliable_topics(self, topics: list):
        self.target_topic = list()
        for i in reversed(range(self.verticalLayout.count())): 
            self.verticalLayout.itemAt(i).widget().setParent(None)
        self.checkBox = list()
        for i, text in enumerate(topics):
            self.checkBox.append(QtWidgets.QCheckBox(parent=self.verticalLayoutWidget))
            self.checkBox[i].setObjectName(text)
            self.checkBox[i].setText(text)
            self.checkBox[i].clicked.connect(self._set_processed_topic_list)
            self.verticalLayout.addWidget(self.checkBox[i])

    def _set_processed_topic_list(self):
        checkbox = self.verticalLayoutWidget.sender()
        if checkbox.isChecked():
            if checkbox.text() not in self.target_topic:
                self.target_topic.append(checkbox.text())
        else:
            if checkbox.text() in self.target_topic:
                self.target_topic.remove(checkbox.text())

    def _onAccepted(self):
        self._check_target_topic()
        print(self.target_topic)
        if not self._check_output():
            return None
        self.progressdialog = QtWidgets.QProgressDialog('Processing', 'Cancel', 0, self.bag.t_end - self.bag.t_start, None)
        self.progressdialog.setWindowModality(QtCore.Qt.WindowModality.WindowModal)
        self._processed_msgs()
        # self.progressdialog.close()
        # QtWidgets.QMessageBox.information(None, 'Processed', 'processing')
        print("finsish")

    def _onCanceled(self):
        QtWidgets.QMessageBox.information(None, 'Canceled', 'Canceled')

    
    def _check_output(self):
        if not self.outdir_ready:
            QtWidgets.QMessageBox.warning(None, "No out dir", "Please select the output directory")
            return False
        self.bag.check_output(self.target_topic)
        return True

    def _check_target_topic(self):
        for checkbox in self.checkBox:
            if not checkbox.isChecked() and checkbox.text() in self.target_topic:
                self.target_topic.remove(checkbox.text())
            elif checkbox.isChecked() and checkbox.text() not in self.target_topic:
                self.target_topic.append(checkbox.text())


    def _processed_msgs(self):
        for topic, msg, t in self.bag.get_msgs(self.target_topic):
            if topic in self.target_topic:
                data = self.converter.convert(msg)
                self.bag.data_output(data)
            self.progressdialog.setValue(t.to_sec() -self.bag.t_start)

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec())
