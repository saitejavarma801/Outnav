import logging
import sys

import rclpy
from PyQt5 import QtWidgets

from logging_utils import setup_logging
from main_window import OutdoorNavApp


def main():
    log_path = setup_logging()
    logging.info("[App] Log capture enabled → %s", log_path)
    logging.info("[App] Booting OutdoorNav...")

    #  1. Initialize ROS2 first
    rclpy.init(args=None)

    #  2. Then create the Qt application
    app = QtWidgets.QApplication(sys.argv)

    #  3. Now create and show your main window
    win = OutdoorNavApp()
    win.show()

    logging.info("[App] OutdoorNav started successfully.")

    #  4. Start the Qt event loop
    exit_code = app.exec_()

    #  5. Cleanly shut down ROS2
    rclpy.shutdown()
    logging.info("[App] Shutdown complete. Exiting with code %s", exit_code)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
