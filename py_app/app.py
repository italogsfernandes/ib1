# -*- coding: utf-8 -*-
# ------------------------------------------------------------------------------
# FEDERAL UNIVERSITY OF UBERLANDIA
# Faculty of Electrical Engineering
# Biomedical Engineering Lab
# ------------------------------------------------------------------------------
# Author: Italo Gustavo Sampaio Fernandes
# Contact: italogsfernandes@gmail.com
# Git: www.github.com/italogfernandes
# ------------------------------------------------------------------------------
# Description:
# ------------------------------------------------------------------------------
from ArduinoEMGPlotter import ArduinoEMGPlotter
import numpy as np
import scipy as sp

import sys
if sys.version_info.major == 3:
    # PyQt5
    from PyQt5.QtWidgets import *
    from views import base_qt5 as base
    from PyQt5 import QtCore
    # from PyQt5.QtCore import SIGNAL
    from views import config_processamento_qt5 as config_window
elif sys.version_info.major == 2:
    # PyQt4
    from PyQt4.QtGui import *
    from views import base_qt4 as base
    from PyQt4 import QtCore
    from PyQt4.QtCore import SIGNAL
    from views import config_processamento_qt4 as config_window
else:
    print("Versao do python nao suportada")
# ------------------------------------------------------------------------------
import matplotlib.pyplot as plt  # Showing images
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT as NavigationToolbar
# ------------------------------------------------------------------------------
from matplotlib.figure import Figure


class SetupApp(QMainWindow, config_window.Ui_windowConfig):
    def __init__(self, parent=None):
        super(SetupApp, self).__init__(parent)
        self.setupUi(self)
        # Define a new signal called 'trigger' that has no arguments.
        #self.trigger = QtCore.pyqtSignal()
        self.setup_signals_connections()
        self.tipos_de_processamento = ['Desativado', 'Simples', 'Plotter', 'Thread']
        self.populate_cb()

    def setup_signals_connections(self):
        self.comboBox.currentIndexChanged.connect(self.setup_changed)

    def populate_cb(self):
        self.comboBox.clear()
        for tipo in self.tipos_de_processamento:
            self.comboBox.addItem(tipo)
        self.comboBox.setCurrentIndex(0)

    def setup_changed(self):
        proc = self.comboBox.itemText(self.comboBox.currentIndex())
        # print("Setup Changed to:")
        # print(proc)
        #self.trigger.emit(SIGNAL("proc_changed(QString)"), proc)

    def closeEvent(self, q_close_event):
        super(self.__class__, self).closeEvent(q_close_event)


class ContractionDetector(QMainWindow, base.Ui_MainWindow):
    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)
        self.setup_signals_connections()

        self.emg_app = ArduinoEMGPlotter(parent=self.centralwidget,label=self.lbl_status)
        self.verticalLayoutGraph.addWidget(self.emg_app.plotHandler.plotWidget)

        self.verticalLayoutGraph.removeWidget(self.label_replace)
        self.label_replace.setParent(None)

        self.cb_emg.toggle()
        self.sl_threshould.setValue(0.25)
        self.sl_threshould_value_changed(10)
        self.proc_changed("Desativado")

        self.edited_image_fig = Figure(figsize=(0.1, 0.1))
        self.edited_image_canvas = FigureCanvas(self.edited_image_fig)
        self.edited_image_toolbar = NavigationToolbar(
            self.edited_image_canvas, self, coordinates=True)

        self.edited_image_canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.edited_image_canvas.updateGeometry()
        self.verticalLayoutGraph.addWidget(self.edited_image_canvas)
        self.edited_image_canvas.draw()
        self.verticalLayoutGraph.addWidget(self.edited_image_toolbar)

        time_example = np.linspace(0, 1, 100)
        y_example = np.sin(2 * np.pi * 3 * time_example)
        gca_example = self.edited_image_fig.gca()
        gca_example.plot(time_example, y_example)
        gca_example.set_title("MMN")
        gca_example.grid()
        gca_example.set_xlabel('Index')
        gca_example.set_ylabel('Media do sinal')

    def setup_signals_connections(self):
        #self.actionProcessamento.triggered.connect(self.processamento_clicked)
        self.btn_start.clicked.connect(self.btn_start_clicked)
        self.btn_calib.clicked.connect(self.btn_calib_clicked)
        self.sl_threshould.valueChanged.connect(self.sl_threshould_value_changed)
        self.cb_emg.toggled.connect(lambda x: self.emg_app.plotHandler.emg_bruto.set_visible(x))
        self.cb_hbt.toggled.connect(lambda x: self.emg_app.plotHandler.hilbert.set_visible(x))
        self.cb_ret.toggled.connect(lambda x: self.emg_app.plotHandler.hilbert_retificado.set_visible(x))
        self.cb_env.toggled.connect(lambda x: self.emg_app.plotHandler.envoltoria.set_visible(x))
        self.cb_lim.toggled.connect(lambda x: self.emg_app.plotHandler.threshold.set_visible(x))
        self.cb_det.toggled.connect(lambda x: self.emg_app.plotHandler.set_detection_visible(x))
        #self.connect(setup_form, SIGNAL("proc_changed(QString)"), self.proc_changed)

    def proc_changed(self, new_proc):
        print(new_proc)
        self.emg_app.update_proc_type(new_proc)

    def closeEvent(self, q_close_event):
        self.emg_app.stop()
        super(self.__class__, self).closeEvent(q_close_event)

    def processamento_clicked(self):
        setup_form.show()

    def btn_start_clicked(self):
        if self.emg_app.started:
            self.emg_app.stop()
            self.btn_start.setText('Start')
        else:
            self.emg_app.start()
            self.btn_start.setText('Stop')

    def btn_calib_clicked(self):
        self.lbl_status.setText(QtCore.QString.fromUtf8("Status: Calibração não implementada."))

    def sl_threshould_value_changed(self, sl_value):
        # self.lbl_threshould.setText("Limiar:\n%.2f" % (sl_value * 1.0 / 100))
        self.emg_app.plotHandler.set_threshold(sl_value * 1.0 / 100)

app = QApplication(sys.argv)
setup_form = SetupApp()
form = ContractionDetector()


def main():
    form.show()
    app.exec_()

if __name__ == "__main__":
    main()
