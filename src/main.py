#!/usr/bin/env python3

import sys
import os

# Add the project root to Python path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from PySide6.QtWidgets import QApplication
from src.windows.launcher_window import LauncherWindow

def main():
    app = QApplication(sys.argv)
    
    # Create and show launcher window
    launcher = LauncherWindow()
    launcher.show()
    
    sys.exit(app.exec())

if __name__ == "__main__":
    main()