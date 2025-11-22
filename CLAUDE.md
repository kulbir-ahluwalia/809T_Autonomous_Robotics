# CLAUDE.md

This file provides guidance to Claude Code when working with this directory.

## Directory Overview

This directory contains course materials for ENPM 809T Autonomous Robotics, a graduate-level course focusing on autonomous mobile robots. The repository includes assignments, lectures, and practical implementations covering locomotion, localization, sensor processing, and control systems.

## Directory Structure

```
809T_Autonomous_Robotics/
├── Assignments/                    # 7 assignments + solutions
│   ├── Ass_1/                     # IMU data processing
│   ├── Ass_2/                     # Sensor calibration
│   ├── Ass_3/                     # Localization algorithms
│   ├── Ass_4/                     # Path planning
│   ├── Ass_5/                     # Control systems
│   ├── Ass_6/                     # Servo control & hardware
│   └── Ass_7/                     # Final project
├── Lectures/                       # Course lecture materials
│   ├── Lecture_1/                 # Introduction
│   ├── Lecture_2/                 # Mathematical foundations
│   ├── Lecture3/                  # Sensors and perception
│   ├── Lecture_4/                 # Planning algorithms
│   ├── Lec5_6_7_Locomotion/      # Robot locomotion (3 lectures)
│   └── Lec8_9_Localization/      # Localization methods (2 lectures)
└── Syllabus/                      # Course syllabus and schedule
```

## Course Topics

### Module 1: Fundamentals
- **Lecture 1**: Introduction to autonomous robotics
- **Lecture 2**: Mathematical foundations (linear algebra, probability)

### Module 2: Perception
- **Lecture 3**: Sensors and perception systems
- IMU, accelerometers, gyroscopes
- Sensor calibration and noise modeling

### Module 3: Locomotion (Lectures 5-7)
- Differential drive kinematics
- Wheel odometry
- Motion models
- Includes experimental data:
  - `0.8m_forward/` - Forward motion tests
  - `0.7m_backward/` - Backward motion tests
  - `left_90_degree/` - Left turn calibration
  - `right_90_degree/` - Right turn calibration

### Module 4: Localization (Lectures 8-9)
- Probabilistic localization
- Kalman filters
- Particle filters
- SLAM fundamentals

### Module 5: Planning & Control
- **Lecture 4**: Path planning algorithms
- Motion planning
- Trajectory optimization

## Assignments Overview

### Assignment 1: IMU Data Processing
- Files: `accel.py`, `imudata.txt`, `adxl327.pdf`
- Process raw IMU data from ADXL327 accelerometer
- Implement calibration and noise filtering
- Jupyter notebook solutions included

### Assignment 2: Sensor Calibration
- Calibrate various sensors
- Error modeling and compensation

### Assignment 3: Localization Implementation
- Includes `FINAL/` subdirectory with complete solution
- Implement EKF or particle filter
- Test with real sensor data

### Assignment 4: Path Planning
- Contains `submission/` folder with final code
- Implement path planning algorithms
- Test in simulated environments

### Assignment 5: Control Systems
- Design and implement robot controllers
- PID tuning and optimization

### Assignment 6: Hardware Integration
- Servo control experiments
- Timestamped data folders:
  - `20210402-221652/`
  - `20210402-221955/`
  - `20210402-222403/`
  - `20210402-222642/`
- `servo_pics/` - Documentation photos

### Assignment 7: Final Project
- Comprehensive project combining all concepts
- Full autonomous system implementation

## File Types & Technologies

### Programming Languages
- **Python**: Primary language (`.py` files)
- **Jupyter Notebooks**: `.ipynb` files for assignments
- PDF exports of notebooks for submission

### Key Libraries/Tools
- NumPy for numerical computation
- Matplotlib for visualization
- ROS for robot communication
- OpenCV for vision processing

## Development Guidelines

### For Students
1. Start with assignments in numerical order
2. Review corresponding lectures before each assignment
3. Test code with provided data files
4. Document experimental results

### Running Assignments
```python
# Example for Assignment 1
python accel.py < imudata.txt

# Or use Jupyter notebooks
jupyter notebook 809T_assign1.ipynb
```

### Data Processing Pattern
Most assignments follow this pattern:
1. Read sensor data from text files
2. Apply calibration/filtering
3. Implement algorithm
4. Visualize results
5. Generate report

## Hardware Components

Based on Assignment 6:
- Servo motors for actuation
- IMU sensors (ADXL327 accelerometer)
- Microcontroller interface
- Data logging systems

## Course Level & Prerequisites

- **Level**: Graduate (809T indicates graduate course)
- **Prerequisites**:
  - Linear algebra
  - Probability and statistics
  - Programming experience
  - Basic robotics knowledge

## Git Repository

This is a git repository (`.git` folder present), suggesting:
- Version control for assignments
- Collaborative development
- Historical tracking of solutions

## Important Notes

### Experimental Data
- Locomotion tests include real robot motion data
- Timestamped folders contain experimental results
- Calibration data for specific robot platforms

### Academic Integrity
- Solutions are included for reference
- Understand concepts before using code
- Cite appropriately if reusing

## Related Courses

This appears related to:
- CS498GC Mobile Robotics (similar topics)
- Complements theoretical knowledge with practical implementation
- Focus on autonomous systems vs. general mobile robotics