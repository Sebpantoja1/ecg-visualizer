import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from services.serial_service import SerialService
from services.display_service import DisplayService


class ECGVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ECG Visualizer")
        self.setGeometry(100, 100, 800, 600)

        # Create services
        self.serial_service = SerialService(port='COM3', baudrate=115200)  # Adjust port as needed
        self.display_service = DisplayService()

        # Connect signals
        self.serial_service.data_received.connect(self.display_service.update_data)

        # Setup UI
        central_widget = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(self.display_service)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        # Start services
        self.serial_service.start()

    def closeEvent(self, event):
        self.serial_service.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = ECGVisualizer()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
