# Makefile for Raspberry Pi Sensor and Motor Control Project

# Phony targets are not associated with files
.PHONY: all install clean run-sample generate-sample-data

# Default target
all: install

# Target to install python dependencies
install:
	pip install -r requirements.txt

# Target to generate sample data for testing
generate-sample-data:
	python example/example_data_generator.py

# Target to run the main script with sample data
# This is the primary quick-run check for verification.
run-sample: generate-sample-data
	python src/main.py --input-remote example/remote_control_sample.txt --input-lidar example/lidar_sample.txt --input-gps example/gps_sample.txt

# Target to run the keyboard simulation mode
run-simulate:
	python src/main.py --simulate

# Target to run the motor test
test-motor:
	python src/motor.py

# Target to clean up log files and python cache
clean:
	rm -rf logs/*.log
	rm -rf src/__pycache__ example/__pycache__

# Note: The 'run-live' functionality is executed directly via python command
# as it's intended for the actual Raspberry Pi hardware environment.
# Example: python src/main.py --live
