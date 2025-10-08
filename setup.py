from setuptools import setup, find_packages

setup(
    name="robot_control_ui",
    version="1.0.0",
    description="Comprehensive Robot Control UI with RViz integration",
    author="Viju",
    author_email="vijithsreevalsam@example.com",
    packages=find_packages(),
    install_requires=[
        "PySide6>=6.4.0",
        "rclpy>=3.3.0",
    ],
    entry_points={
        "console_scripts": [
            "robot_control_ui=src.main:main",
        ],
    },
    python_requires=">=3.8",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
)