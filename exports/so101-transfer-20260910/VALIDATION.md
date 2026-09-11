# 迁移副本验证

本机 Linux，Python 3.12.14、MuJoCo 3.3.2，使用原环境解释器提供第三方依赖，工作目录切换到独立迁移副本，其 PYTHONPATH 仅指定副本中的 Nexus 源码。

命令：`PYTHONPATH=third_party/so101-nexus/src MUJOCO_GL=egl /原环境/bin/python -m pytest tests -q --confcutdir=. --rootdir=. --import-mode=importlib`

结果：42 passed, 3 skipped。跳过的是需要桌面的图形测试；包括新闭环真实服务、运动、重置、日志及 RGB-D 定位的其余测试通过。未在第二台物理电脑或其他操作系统上执行。

MANIFEST.sha256 给出本包各文件的 SHA-256（不含清单自身）；压缩包外另有 .sha256 校验文件。
