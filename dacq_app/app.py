# -*- coding: utf-8 -*-
# ------------------------------------------------------------------------------
# FEDERAL UNIVERSITY OF UBERLANDIA
# Faculty of Electrical Engineering
# ------------------------------------------------------------------------------
# Author: Italo Gustavo Sampaio Fernandes
# Contact: italogsfernandes@gmail.com
# Git: www.github.com/italogfernandes
# ------------------------------------------------------------------------------
# Description:
# ------------------------------------------------------------------------------
import sys
if sys.version_info.major == 3:
    # PyQt5
    from PyQt5.QtWidgets import *
    from views import base_qt5 as base
    from PyQt5 import QtCore
elif sys.version_info.major == 2:
    # PyQt4
    from PyQt4.QtGui import *
    from views import base_qt4 as base
    from PyQt4 import QtCore
else:
    print("Versao do python nao suportada")
# ------------------------------------------------------------------------------


class IB1AppForm(QMainWindow, base.Ui_MainWindow):
    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        self.setupUi(self)
        self.setup_signals_connections()

    def setup_signals_connections(self):
        pass

    def closeEvent(self, q_close_event):
        pass
        super(self.__class__, self).closeEvent(q_close_event)


app = QApplication(sys.argv)
form = IB1AppForm()


def main():
    form.show()
    app.exec_()

if __name__ == "__main__":
    main()
