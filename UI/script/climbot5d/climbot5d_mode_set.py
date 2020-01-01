# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'climbot5d_Mode_Set.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_climbot5d_Mode_Set(object):
    def setupUi(self, climbot5d_Mode_Set):
        climbot5d_Mode_Set.setObjectName("climbot5d_Mode_Set")
        climbot5d_Mode_Set.resize(240, 320)
        self.form = climbot5d_Mode_Set
        self.pushButton = QtWidgets.QPushButton(climbot5d_Mode_Set)
        self.pushButton.setGeometry(QtCore.QRect(50, 50, 140, 50))
        self.pushButton.setObjectName("pushButton")
        # self.pushButton_2 = QtWidgets.QPushButton(climbot5d_Mode_Set)
        # self.pushButton_2.setGeometry(QtCore.QRect(50, 130, 140, 50))
        # self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_3 = QtWidgets.QPushButton(climbot5d_Mode_Set)
        self.pushButton_3.setGeometry(QtCore.QRect(0, 0, 51, 25))
        self.pushButton_3.setObjectName("pushButton_3")
        # self.pushButton_4 = QtWidgets.QPushButton(climbot5d_Mode_Set)
        # self.pushButton_4.setGeometry(QtCore.QRect(50, 220, 140, 50))
        # self.pushButton_4.setObjectName("pushButton_4")

        self.retranslateUi(climbot5d_Mode_Set)
        self.pushButton.clicked.connect(climbot5d_Mode_Set.joint_control)
        # self.pushButton_2.clicked.connect(climbot5d_Mode_Set.decartes_control)
        self.pushButton_3.clicked.connect(climbot5d_Mode_Set.return_last_ui)
        # self.pushButton_4.clicked.connect(climbot5d_Mode_Set.data_show)
        QtCore.QMetaObject.connectSlotsByName(climbot5d_Mode_Set)

    def retranslateUi(self, climbot5d_Mode_Set):
        _translate = QtCore.QCoreApplication.translate
        climbot5d_Mode_Set.setWindowTitle(_translate("climbot5d_Mode_Set", "模式设置"))
        self.pushButton.setText(_translate("climbot5d_Mode_Set", "关节空间控制"))
        # self.pushButton_2.setText(_translate("climbot5d_Mode_Set", "笛卡尔空间控制"))
        self.pushButton_3.setText(_translate("climbot5d_Mode_Set", "返回"))
        # self.pushButton_4.setText(_translate("climbot5d_Mode_Set", "数据再现"))



