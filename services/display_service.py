import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtCore import pyqtSlot
import numpy as np


class DisplayService(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.data_buffer = []
        self.max_points = 1000  # Number of points to display

    def init_ui(self):
        layout = QVBoxLayout()
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setTitle("ECG ADC Values")
        self.plot_widget.setLabel('left', 'ADC Value')
        self.plot_widget.setLabel('bottom', 'Sample')
        self.curve = self.plot_widget.plot(pen='y')
        layout.addWidget(self.plot_widget)
        self.setLayout(layout)

    @pyqtSlot(int)
    def update_data(self, adc_value):
        self.data_buffer.append(adc_value)
        if len(self.data_buffer) > self.max_points:
            self.data_buffer.pop(0)
        self.curve.setData(np.arange(len(self.data_buffer)), self.data_buffer)