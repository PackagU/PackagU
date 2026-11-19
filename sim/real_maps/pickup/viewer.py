#!/usr/bin/env python3
"""Observation-only Qt viewer, in its own main thread and without host IPC sharing."""
import os
import sys
import time
from pathlib import Path

os.environ['QT_X11_NO_MITSHM'] = '1'
from PyQt5 import QtCore, QtGui, QtWidgets


def main():
    path = Path(sys.argv[1])
    app = QtWidgets.QApplication(sys.argv)
    widget = QtWidgets.QWidget()
    widget.setWindowTitle('PackagU pickup - rendered body camera / ArUco')
    layout = QtWidgets.QVBoxLayout(widget)
    image, status = QtWidgets.QLabel('Waiting for rendered camera'), QtWidgets.QLabel()
    layout.addWidget(image)
    layout.addWidget(status)
    widget.resize(660, 530)
    widget.move(1300, 70)

    def refresh():
        try:
            pixels = QtGui.QImage.fromData(path.read_bytes())
            if not pixels.isNull():
                image.setPixmap(QtGui.QPixmap.fromImage(pixels))
            age = time.time()-path.stat().st_mtime
            status.setText('Rendered image + ArUco pocket estimate | age %.1fs%s' %
                           (age, ' | NO NEW FRAME' if age > 1.0 else ''))
        except (FileNotFoundError, OSError):
            status.setText('Waiting for camera image')

    timer = QtCore.QTimer()
    timer.timeout.connect(refresh)
    timer.start(150)
    widget.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
