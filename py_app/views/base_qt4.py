# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'layouts/base.ui'
#
# Created by: PyQt4 UI code generator 4.12.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(800, 576)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        MainWindow.setMinimumSize(QtCore.QSize(0, 0))
        MainWindow.setMaximumSize(QtCore.QSize(16777215, 16777215))
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.verticalLayoutGraphStatus = QtGui.QVBoxLayout()
        self.verticalLayoutGraphStatus.setObjectName(_fromUtf8("verticalLayoutGraphStatus"))
        self.verticalLayoutGraph = QtGui.QVBoxLayout()
        self.verticalLayoutGraph.setObjectName(_fromUtf8("verticalLayoutGraph"))
        self.label_replace = QtGui.QLabel(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_replace.sizePolicy().hasHeightForWidth())
        self.label_replace.setSizePolicy(sizePolicy)
        self.label_replace.setObjectName(_fromUtf8("label_replace"))
        self.verticalLayoutGraph.addWidget(self.label_replace)
        self.verticalLayoutGraphStatus.addLayout(self.verticalLayoutGraph)
        self.lbl_status = QtGui.QLabel(self.centralwidget)
        self.lbl_status.setObjectName(_fromUtf8("lbl_status"))
        self.verticalLayoutGraphStatus.addWidget(self.lbl_status)
        self.horizontalLayout.addLayout(self.verticalLayoutGraphStatus)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 23))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        self.menuArquivo = QtGui.QMenu(self.menubar)
        self.menuArquivo.setObjectName(_fromUtf8("menuArquivo"))
        self.menuHelp = QtGui.QMenu(self.menubar)
        self.menuHelp.setObjectName(_fromUtf8("menuHelp"))
        self.menuConfiguracoes = QtGui.QMenu(self.menubar)
        self.menuConfiguracoes.setObjectName(_fromUtf8("menuConfiguracoes"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)
        self.dockWidget = QtGui.QDockWidget(MainWindow)
        self.dockWidget.setObjectName(_fromUtf8("dockWidget"))
        self.dockWidgetContents = QtGui.QWidget()
        self.dockWidgetContents.setObjectName(_fromUtf8("dockWidgetContents"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout(self.dockWidgetContents)
        self.horizontalLayout_2.setMargin(0)
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.verticalLayoutOptions = QtGui.QVBoxLayout()
        self.verticalLayoutOptions.setObjectName(_fromUtf8("verticalLayoutOptions"))
        self.tabWidget = QtGui.QTabWidget(self.dockWidgetContents)
        self.tabWidget.setObjectName(_fromUtf8("tabWidget"))
        self.tab = QtGui.QWidget()
        self.tab.setObjectName(_fromUtf8("tab"))
        self.verticalLayout_2 = QtGui.QVBoxLayout(self.tab)
        self.verticalLayout_2.setMargin(0)
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.verticalLayout_4 = QtGui.QVBoxLayout()
        self.verticalLayout_4.setContentsMargins(-1, 0, -1, -1)
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.label_8 = QtGui.QLabel(self.tab)
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.verticalLayout_4.addWidget(self.label_8)
        self.horizontalSlider_3 = QtGui.QSlider(self.tab)
        self.horizontalSlider_3.setProperty("value", 40)
        self.horizontalSlider_3.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_3.setObjectName(_fromUtf8("horizontalSlider_3"))
        self.verticalLayout_4.addWidget(self.horizontalSlider_3)
        self.label_6 = QtGui.QLabel(self.tab)
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.verticalLayout_4.addWidget(self.label_6)
        self.horizontalSlider = QtGui.QSlider(self.tab)
        self.horizontalSlider.setMaximum(50)
        self.horizontalSlider.setProperty("value", 5)
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setObjectName(_fromUtf8("horizontalSlider"))
        self.verticalLayout_4.addWidget(self.horizontalSlider)
        self.label_7 = QtGui.QLabel(self.tab)
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.verticalLayout_4.addWidget(self.label_7)
        self.horizontalSlider_2 = QtGui.QSlider(self.tab)
        self.horizontalSlider_2.setProperty("value", 20)
        self.horizontalSlider_2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_2.setObjectName(_fromUtf8("horizontalSlider_2"))
        self.verticalLayout_4.addWidget(self.horizontalSlider_2)
        self.verticalLayout_2.addLayout(self.verticalLayout_4)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem)
        self.tabWidget.addTab(self.tab, _fromUtf8(""))
        self.tab_2 = QtGui.QWidget()
        self.tab_2.setObjectName(_fromUtf8("tab_2"))
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.tab_2)
        self.verticalLayout_3.setMargin(0)
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.horizontalLayoutBtns = QtGui.QHBoxLayout()
        self.horizontalLayoutBtns.setObjectName(_fromUtf8("horizontalLayoutBtns"))
        spacerItem1 = QtGui.QSpacerItem(0, 20, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Minimum)
        self.horizontalLayoutBtns.addItem(spacerItem1)
        self.verticalLayoutBtns = QtGui.QVBoxLayout()
        self.verticalLayoutBtns.setObjectName(_fromUtf8("verticalLayoutBtns"))
        self.btn_calib = QtGui.QPushButton(self.tab_2)
        self.btn_calib.setMaximumSize(QtCore.QSize(80, 16777215))
        self.btn_calib.setObjectName(_fromUtf8("btn_calib"))
        self.verticalLayoutBtns.addWidget(self.btn_calib)
        self.horizontalLayoutBtns.addLayout(self.verticalLayoutBtns)
        spacerItem2 = QtGui.QSpacerItem(0, 20, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Minimum)
        self.horizontalLayoutBtns.addItem(spacerItem2)
        self.verticalLayout_3.addLayout(self.horizontalLayoutBtns)
        self.horizontalLayoutThreshould = QtGui.QHBoxLayout()
        self.horizontalLayoutThreshould.setObjectName(_fromUtf8("horizontalLayoutThreshould"))
        spacerItem3 = QtGui.QSpacerItem(0, 20, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Minimum)
        self.horizontalLayoutThreshould.addItem(spacerItem3)
        self.sl_threshould = QtGui.QSlider(self.tab_2)
        self.sl_threshould.setEnabled(True)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sl_threshould.sizePolicy().hasHeightForWidth())
        self.sl_threshould.setSizePolicy(sizePolicy)
        self.sl_threshould.setOrientation(QtCore.Qt.Vertical)
        self.sl_threshould.setObjectName(_fromUtf8("sl_threshould"))
        self.horizontalLayoutThreshould.addWidget(self.sl_threshould)
        spacerItem4 = QtGui.QSpacerItem(0, 20, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Minimum)
        self.horizontalLayoutThreshould.addItem(spacerItem4)
        self.verticalLayout_3.addLayout(self.horizontalLayoutThreshould)
        self.verticalLayoutCheckBoxes = QtGui.QVBoxLayout()
        self.verticalLayoutCheckBoxes.setObjectName(_fromUtf8("verticalLayoutCheckBoxes"))
        self.cb_emg = QtGui.QCheckBox(self.tab_2)
        self.cb_emg.setObjectName(_fromUtf8("cb_emg"))
        self.verticalLayoutCheckBoxes.addWidget(self.cb_emg)
        self.cb_hbt = QtGui.QCheckBox(self.tab_2)
        self.cb_hbt.setObjectName(_fromUtf8("cb_hbt"))
        self.verticalLayoutCheckBoxes.addWidget(self.cb_hbt)
        self.cb_ret = QtGui.QCheckBox(self.tab_2)
        self.cb_ret.setObjectName(_fromUtf8("cb_ret"))
        self.verticalLayoutCheckBoxes.addWidget(self.cb_ret)
        self.cb_env = QtGui.QCheckBox(self.tab_2)
        self.cb_env.setObjectName(_fromUtf8("cb_env"))
        self.verticalLayoutCheckBoxes.addWidget(self.cb_env)
        self.cb_lim = QtGui.QCheckBox(self.tab_2)
        self.cb_lim.setObjectName(_fromUtf8("cb_lim"))
        self.verticalLayoutCheckBoxes.addWidget(self.cb_lim)
        self.cb_det = QtGui.QCheckBox(self.tab_2)
        self.cb_det.setObjectName(_fromUtf8("cb_det"))
        self.verticalLayoutCheckBoxes.addWidget(self.cb_det)
        self.verticalLayout_3.addLayout(self.verticalLayoutCheckBoxes)
        self.tabWidget.addTab(self.tab_2, _fromUtf8(""))
        self.verticalLayoutOptions.addWidget(self.tabWidget)
        spacerItem5 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayoutOptions.addItem(spacerItem5)
        self.verticalLayoutThreshould = QtGui.QVBoxLayout()
        self.verticalLayoutThreshould.setObjectName(_fromUtf8("verticalLayoutThreshould"))
        self.verticalLayout_6 = QtGui.QVBoxLayout()
        self.verticalLayout_6.setContentsMargins(-1, 0, -1, -1)
        self.verticalLayout_6.setObjectName(_fromUtf8("verticalLayout_6"))
        self.label_10 = QtGui.QLabel(self.dockWidgetContents)
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.verticalLayout_6.addWidget(self.label_10)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setContentsMargins(-1, 0, -1, -1)
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.checkBox = QtGui.QCheckBox(self.dockWidgetContents)
        self.checkBox.setObjectName(_fromUtf8("checkBox"))
        self.horizontalLayout_3.addWidget(self.checkBox)
        self.checkBox_2 = QtGui.QCheckBox(self.dockWidgetContents)
        self.checkBox_2.setObjectName(_fromUtf8("checkBox_2"))
        self.horizontalLayout_3.addWidget(self.checkBox_2)
        self.verticalLayout_6.addLayout(self.horizontalLayout_3)
        self.verticalLayoutThreshould.addLayout(self.verticalLayout_6)
        self.verticalLayout_5 = QtGui.QVBoxLayout()
        self.verticalLayout_5.setContentsMargins(-1, 0, -1, -1)
        self.verticalLayout_5.setObjectName(_fromUtf8("verticalLayout_5"))
        self.label_9 = QtGui.QLabel(self.dockWidgetContents)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_9.setFont(font)
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.verticalLayout_5.addWidget(self.label_9)
        self.checkBox_3 = QtGui.QCheckBox(self.dockWidgetContents)
        self.checkBox_3.setObjectName(_fromUtf8("checkBox_3"))
        self.verticalLayout_5.addWidget(self.checkBox_3)
        self.checkBox_4 = QtGui.QCheckBox(self.dockWidgetContents)
        self.checkBox_4.setObjectName(_fromUtf8("checkBox_4"))
        self.verticalLayout_5.addWidget(self.checkBox_4)
        self.checkBox_5 = QtGui.QCheckBox(self.dockWidgetContents)
        self.checkBox_5.setObjectName(_fromUtf8("checkBox_5"))
        self.verticalLayout_5.addWidget(self.checkBox_5)
        self.verticalLayoutThreshould.addLayout(self.verticalLayout_5)
        self.btn_start = QtGui.QPushButton(self.dockWidgetContents)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btn_start.sizePolicy().hasHeightForWidth())
        self.btn_start.setSizePolicy(sizePolicy)
        self.btn_start.setMinimumSize(QtCore.QSize(80, 0))
        self.btn_start.setMaximumSize(QtCore.QSize(12312333, 16777215))
        self.btn_start.setObjectName(_fromUtf8("btn_start"))
        self.verticalLayoutThreshould.addWidget(self.btn_start)
        self.verticalLayoutOptions.addLayout(self.verticalLayoutThreshould)
        self.horizontalLayout_2.addLayout(self.verticalLayoutOptions)
        self.dockWidget.setWidget(self.dockWidgetContents)
        MainWindow.addDockWidget(QtCore.Qt.DockWidgetArea(1), self.dockWidget)
        self.dockWidget_2 = QtGui.QDockWidget(MainWindow)
        self.dockWidget_2.setObjectName(_fromUtf8("dockWidget_2"))
        self.dockWidgetContents_2 = QtGui.QWidget()
        self.dockWidgetContents_2.setObjectName(_fromUtf8("dockWidgetContents_2"))
        self.verticalLayout = QtGui.QVBoxLayout(self.dockWidgetContents_2)
        self.verticalLayout.setMargin(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.label_3 = QtGui.QLabel(self.dockWidgetContents_2)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.verticalLayout.addWidget(self.label_3)
        self.label_4 = QtGui.QLabel(self.dockWidgetContents_2)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.verticalLayout.addWidget(self.label_4)
        spacerItem6 = QtGui.QSpacerItem(58, 433, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem6)
        self.dockWidget_2.setWidget(self.dockWidgetContents_2)
        MainWindow.addDockWidget(QtCore.Qt.DockWidgetArea(2), self.dockWidget_2)
        self.actionSimples = QtGui.QAction(MainWindow)
        self.actionSimples.setObjectName(_fromUtf8("actionSimples"))
        self.actionIn_Plotter = QtGui.QAction(MainWindow)
        self.actionIn_Plotter.setObjectName(_fromUtf8("actionIn_Plotter"))
        self.actionThread = QtGui.QAction(MainWindow)
        self.actionThread.setObjectName(_fromUtf8("actionThread"))
        self.actionDesativado = QtGui.QAction(MainWindow)
        self.actionDesativado.setObjectName(_fromUtf8("actionDesativado"))
        self.actionProcessamento = QtGui.QAction(MainWindow)
        self.actionProcessamento.setCheckable(False)
        self.actionProcessamento.setObjectName(_fromUtf8("actionProcessamento"))
        self.menuConfiguracoes.addAction(self.actionProcessamento)
        self.menubar.addAction(self.menuArquivo.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())
        self.menubar.addAction(self.menuConfiguracoes.menuAction())

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "EEG App - Instrumentação Biomédica 1", None))
        self.label_replace.setText(_translate("MainWindow", "Chart Here", None))
        self.lbl_status.setText(_translate("MainWindow", "Status:", None))
        self.menuArquivo.setTitle(_translate("MainWindow", "Arquivo", None))
        self.menuHelp.setTitle(_translate("MainWindow", "Help", None))
        self.menuConfiguracoes.setTitle(_translate("MainWindow", "Configurações", None))
        self.label_8.setText(_translate("MainWindow", "Window Size: 100ms", None))
        self.label_6.setText(_translate("MainWindow", "High-pass: 0.5 Hz", None))
        self.label_7.setText(_translate("MainWindow", "Low-pass: 40 Hz", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "Opções", None))
        self.btn_calib.setToolTip(_translate("MainWindow", "Calibrar", None))
        self.btn_calib.setText(_translate("MainWindow", "Procurar", None))
        self.cb_emg.setText(_translate("MainWindow", "Show Trigger", None))
        self.cb_hbt.setText(_translate("MainWindow", "Op 2", None))
        self.cb_ret.setText(_translate("MainWindow", "Op 3", None))
        self.cb_env.setText(_translate("MainWindow", "--", None))
        self.cb_lim.setText(_translate("MainWindow", "--", None))
        self.cb_det.setText(_translate("MainWindow", "--", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "+", None))
        self.label_10.setText(_translate("MainWindow", "Canais", None))
        self.checkBox.setText(_translate("MainWindow", "CH1", None))
        self.checkBox_2.setText(_translate("MainWindow", "CH2", None))
        self.label_9.setText(_translate("MainWindow", "Processamento", None))
        self.checkBox_3.setText(_translate("MainWindow", "Promediação", None))
        self.checkBox_4.setText(_translate("MainWindow", "MNP", None))
        self.checkBox_5.setText(_translate("MainWindow", "P300", None))
        self.btn_start.setToolTip(_translate("MainWindow", "Start", None))
        self.btn_start.setText(_translate("MainWindow", "Start/Stop", None))
        self.label_3.setText(_translate("MainWindow", "Output", None))
        self.label_4.setText(_translate("MainWindow", "SIM", None))
        self.actionSimples.setText(_translate("MainWindow", "Processamento", None))
        self.actionIn_Plotter.setText(_translate("MainWindow", "In Plotter", None))
        self.actionThread.setText(_translate("MainWindow", "Thread", None))
        self.actionDesativado.setText(_translate("MainWindow", "Desativado", None))
        self.actionProcessamento.setText(_translate("MainWindow", "A implementar", None))

