# Multi Robot Challenge 2023

## Table of Contents
- [Introduction](#introduction)
- [Installation](#installation)
- [Usage](#usage)

## Introduction
This project is the semester project of group 30 in the DAT160 course. Due to time constraints, the project is unfortunatly not complete.

## Installation
1. Clone the repository:
    ```sh
    git clone https://github.com/yourusername/multi_robot_challenge_23.git
    ```
2. Navigate to the project directory:
    ```sh
    cd multi_robot_challenge_23
    ```
## Usage
To launch the simulation, use the following command. Substitute X with a number 1-5. Note that the "meeting at big fire"-behaviour only works for world 1:
```sh
ros2 launch multi_robot_challenge_23 rescue_robots_wX.launch.py
```
After the simualtion has launched and has had some time to initalize the services, run the main script in a seperate terminal:
```sh
ros2 run multi_robot_challenge_23 main_controller
```

You can view the system in action here:
https://www.youtube.com/watch?v=L6QqqiK6YXU&ab_channel=%C3%85smundVassbotn


