# Project Overview

This is a robot experiment project. The work includes:

1. Writing code
2. Completing experiment reports

# Project Structure

This project contains three robot types:

- `Delta` parallel robot
- `3-RRR` planar parallel robot
- `FiveBar` planar five-bar parallel robot

Each robot has its own folder:

- `Delta/`
- `3-RRR/`
- `FiveBar/`

# Experiment Content

For all three robots, the experiment content is the same:

Use Python to control the robot through a USB-to-CAN module so that the end-effector pen tip moves to target positions or follows a planned trajectory.

For each robot, the following trajectories are required:

1. Concentric circles
2. Rectangle
3. Square
4. Equilateral triangle

# Tasks

## 1. Code Writing

In each robot folder, the main control file is `test.py`.
This is the file that should be modified for the experiment.
Other Python files are library files and should not be modified unless explicitly required.

The planar five-bar robot (`FiveBar`) already contains a completed example in:

- `./FiveBar/test.py`

Inside that file:

- `rectangle` means rectangle or square, depending on side length
- `circles` means concentric circles
- `triangle` means equilateral triangle

Use the `FiveBar` implementation as the reference and complete the code for:

- `3-RRR/test.py`
- `Delta/test.py`

## 2. Experiment Report

# Report Files

In each project:

1. Files in `Report/`:
   - `*.docx` are the original report files
   - `*.md` are simplified Markdown copies for easier Agent reading
2. Files in `Img/` are experiment photos

# Report Requirements

1. The experiment report must be written in LaTeX and compiled into `.pdf`
2. Complete the sections:
   - `实验数据记录` / Experimental Data Record
   - `程序附录` / Program Appendix
3. The experiment photos in `Img/` must be inserted under the `附图` section
4. Further edit the code according to the report requirements when needed
5. Make sure Chinese LaTeX support works correctly

# Notes

At the project root:

- `实验一.docx` is about motion and mechanism analysis of the three robots and does not need to be completed now
- `实验指导书.docx` is the experiment guide, but it has not been converted into Markdown

# Data Table Format Example

Use the following structure in the report.

Centered title:

`实验数据表`

Inside the table:

1. A full-width row for the experiment item
2. A full-width row for `程序运行的参数`
3. Then the data rows

Example:

1、控制平面3-RRR并联机器人末端画同心圆。  
(this line should occupy one full row inside the table)

程序运行的参数  
(this line should occupy one full row inside the table)

| 圆A起始点坐标值 | 圆A轨迹点数 | 圆A运行时间 |
|---|---:|---|
| data | data | data |

| 圆B起始点坐标值 | 圆B轨迹点数 | 圆B运行时间 |
|---|---:|---|
| data | data | data |

2、控制平面3-RRR并联机器人末端画正三角形。  
(this line should occupy one full row inside the table)

程序运行的参数  
(this line should occupy one full row inside the table)

| 三角形起始点坐标值 | 三角形轨迹点数 | 三角形运行时间 |
|---|---:|---|
| data | data | data |

3、控制平面3-RRR并联机器人末端画矩形。（选）  
(this line should occupy one full row inside the table)

程序运行的参数  
(this line should occupy one full row inside the table)

| 矩形起始点坐标值 | 矩形轨迹点数 | 矩形运行时间 |
|---|---:|---|
| data | data | data |

| 正方形起始点坐标值 | 正方形轨迹点数 | 正方形运行时间 |
|---|---:|---|
| data | data | data |

# Recommended Agent Workflow

When using an Agent on this repository:

1. Read this `AGENT.md` first
2. Read the target robot folder
3. Modify only the necessary `test.py` file unless the report requires more
4. Use images from `Img/`
5. Update the LaTeX report in `Report/`
6. Compile the LaTeX report into PDF
