# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'simple.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(753, 398)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName("gridLayout")
        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.btn_rotate_left = QtWidgets.QPushButton(self.centralwidget)
        self.btn_rotate_left.setObjectName("btn_rotate_left")
        self.gridLayout_2.addWidget(self.btn_rotate_left, 1, 1, 1, 1)
        self.btn_backward = QtWidgets.QPushButton(self.centralwidget)
        self.btn_backward.setObjectName("btn_backward")
        self.gridLayout_2.addWidget(self.btn_backward, 1, 3, 1, 1)
        self.btn_up = QtWidgets.QPushButton(self.centralwidget)
        self.btn_up.setObjectName("btn_up")
        self.gridLayout_2.addWidget(self.btn_up, 8, 3, 1, 1)
        self.btn_tilt_backward = QtWidgets.QPushButton(self.centralwidget)
        self.btn_tilt_backward.setObjectName("btn_tilt_backward")
        self.gridLayout_2.addWidget(self.btn_tilt_backward, 7, 3, 1, 1)
        self.btn_rotate_right = QtWidgets.QPushButton(self.centralwidget)
        self.btn_rotate_right.setObjectName("btn_rotate_right")
        self.gridLayout_2.addWidget(self.btn_rotate_right, 1, 4, 1, 1)
        self.btn_forward = QtWidgets.QPushButton(self.centralwidget)
        self.btn_forward.setEnabled(True)
        self.btn_forward.setObjectName("btn_forward")
        self.gridLayout_2.addWidget(self.btn_forward, 0, 3, 1, 1)
        self.btn_down = QtWidgets.QPushButton(self.centralwidget)
        self.btn_down.setObjectName("btn_down")
        self.gridLayout_2.addWidget(self.btn_down, 8, 4, 1, 1)
        self.btn_stand_mode = QtWidgets.QPushButton(self.centralwidget)
        self.btn_stand_mode.setObjectName("btn_stand_mode")
        self.gridLayout_2.addWidget(self.btn_stand_mode, 8, 1, 1, 1)
        self.btn_tilt_forward = QtWidgets.QPushButton(self.centralwidget)
        self.btn_tilt_forward.setObjectName("btn_tilt_forward")
        self.gridLayout_2.addWidget(self.btn_tilt_forward, 5, 3, 1, 1)
        self.btn_tilt_center = QtWidgets.QPushButton(self.centralwidget)
        self.btn_tilt_center.setObjectName("btn_tilt_center")
        self.gridLayout_2.addWidget(self.btn_tilt_center, 6, 3, 1, 1)
        self.btn_tilt_left = QtWidgets.QPushButton(self.centralwidget)
        self.btn_tilt_left.setObjectName("btn_tilt_left")
        self.gridLayout_2.addWidget(self.btn_tilt_left, 6, 1, 1, 1)
        self.btn_tilt_right = QtWidgets.QPushButton(self.centralwidget)
        self.btn_tilt_right.setObjectName("btn_tilt_right")
        self.gridLayout_2.addWidget(self.btn_tilt_right, 6, 4, 1, 1)
        self.gridLayout.addLayout(self.gridLayout_2, 1, 1, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Diablo Control GUI"))
        self.btn_rotate_left.setText(_translate("MainWindow", "Rotate Left"))
        self.btn_backward.setText(_translate("MainWindow", "Move Backward"))
        self.btn_up.setText(_translate("MainWindow", "Up"))
        self.btn_tilt_backward.setText(_translate("MainWindow", "Tilt Backward"))
        self.btn_rotate_right.setText(_translate("MainWindow", "Rotate Right"))
        self.btn_forward.setText(_translate("MainWindow", "Move Forward"))
        self.btn_down.setText(_translate("MainWindow", "Down"))
        self.btn_stand_mode.setText(_translate("MainWindow", "Stand Mode"))
        self.btn_tilt_forward.setText(_translate("MainWindow", "Tilt Forward"))
        self.btn_tilt_center.setText(_translate("MainWindow", "Center"))
        self.btn_tilt_left.setText(_translate("MainWindow", "Tilt Left"))
        self.btn_tilt_right.setText(_translate("MainWindow", "Tilt Right"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
