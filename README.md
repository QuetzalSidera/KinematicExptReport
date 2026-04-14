# 机器人实验报告辅助项目

本项目用于借助 Agent 完成机器人实验代码整理与实验报告编写。

支持的实验项目包括：

- `FiveBar`：平面五连杆并联机器人系统控制
- `3-RRR`：平面 3-RRR 并联机器人系统控制
- `Delta`：Delta 并联机器人系统控制

本仓库的核心用途是：

1. 将你的实验代码放入对应项目目录
2. 将实验图片放入对应项目的 `Img/` 目录
3. 使用 Agent 参考 `AGENT.md` 自动补全报告
4. 在 `Report/` 目录中找到生成的报告，并继续补充姓名、学号等最终信息

## 1. 配置

使用本项目前，需要准备以下环境：

- `Git`
- `LaTeX` 环境
- `Agent` 环境

推荐的 LaTeX 配置：

- `xelatex`
- `ctex` 中文支持

如果你希望 Agent 可以直接在仓库中完成代码与报告修改，还需要准备可用的 Agent 工具环境，例如：

- `Codex`
- `Claude`

## 2. 使用方法

### 2.1 获取项目

```bash
git clone https://github.com/QuetzalSidera/KinematicExptReport.git
cd RobotCourse
```

### 2.2 替换代码与图像

将你的实验代码覆盖到对应项目中的主控制文件，一般是：

- `FiveBar/test.py`
- `3-RRR/test.py`
- `Delta/test.py`

将实验图片放到对应目录：

- `FiveBar/Img/`
- `3-RRR/Img/`
- `Delta/Img/`

### 2.3 启动 Agent

如果你使用 Codex：

```bash
codex .
```

如果你使用 Claude（按你的本地安装方式启动）：

```bash
claude
```

启动后，可以直接告诉 Agent：

```text
请参考 AGENT.md 中的要求，完成对应机器人的实验报告。
```

### 2.4 查看报告输出

报告输出位于各项目的 `Report/` 目录中，例如：

- `FiveBar/Report/`
- `3-RRR/Report/`
- `Delta/Report/`

你可以在这些目录中找到：

- `.tex` 报告源码
- `.pdf` 编译结果
- `.md` 辅助阅读版本

### 2.5 最后人工补充

在 Agent 完成报告后，建议你继续检查并补充以下内容：

1. 姓名
2. 学号
3. 实验时间与地点
4. 教师信息
5. 将内容整理并补充回课程原有报告模板

## 3. 目录说明

### FiveBar

- 实验名称：平面五连杆并联机器人系统控制
- 代码入口：`FiveBar/test.py`
- 图片目录：`FiveBar/Img/`
- 报告目录：`FiveBar/Report/`

### 3-RRR

- 实验名称：平面 3-RRR 并联机器人系统控制
- 代码入口：`3-RRR/test.py`
- 图片目录：`3-RRR/Img/`
- 报告目录：`3-RRR/Report/`

### Delta

- 实验名称：Delta 并联机器人系统控制
- 代码入口：`Delta/test.py`
- 图片目录：`Delta/Img/`
- 报告目录：`Delta/Report/`

## 4. 说明

- 各项目中的 `test.py` 是主要修改文件。
- 其他 Python 文件通常为库文件，不建议随意修改。
- `Report/*.md` 用于方便 Agent 阅读和整理内容。
- `Report/*.docx` 是课程原始实验报告文件。
- `AGENT.md` 中给出了 Agent 完成任务时应遵循的说明。
