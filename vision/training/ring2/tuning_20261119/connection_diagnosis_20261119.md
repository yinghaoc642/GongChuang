# 控制连接诊断记录

本次用户要求自主修复 Chrome / CUA 连接。结果：尚未恢复，未提交新的 MaixHub 训练。

## 已实测

- CUA 最终返回 `apps: []`、`browsers: []`，并报告：
  - `Sky Computer Use native pipe startup failed`
  - `The browser access host is unavailable`
- 当前 CUA 配置引用 `/Applications/ChatGPT.app`，相关路径存在，未证实“仍引用已不存在的 Codex.app”是本次故障根因。
- 安装的辅助服务与当前应用自带副本的 173 个文件 SHA-256 和权限逐项一致。
- 原生服务的 `computeruse.sock` 存在，`lsof` 显示由 `SkyComputerUseService` 对应进程占用。文件存在和进程运行并不代表 CUA 握手成功。
- 尝试通过 Launch Services 重新唤起辅助服务失败，原始输出：
  `NSOSStatusErrorDomain Code=-10827 kLSNoExecutableErr: The executable is missing`。
  实际主二进制文件存在且可执行，故此错误不能直接解释为文件被删除。
- 对已确认占用该套接字的辅助进程发送 TERM 失败：
  `kill 27480 failed: operation not permitted`。
- 系统日志查询被当前执行环境拒绝：`log: Cannot run while sandboxed`。
- 安装副本及应用自带副本的 `codesign --verify --deep --strict` 均返回：
  `invalid signature (code or signature have been modified)`。
  两份制品逐项一致，且当前检查受执行环境限制；此项不足以确诊本次连接故障，未修改签名或系统安全设置。

## 结论与限制

已排除安装副本缺失、与应用自带副本不一致，以及当前 CUA 主配置仍指向旧主应用路径这几种解释。
精确根因仍未确认；当前证据将问题定位到桌面应用提供的控制通信链路，无法据此归因于 MaixHub 登录或模型参数。

自动修复所需的针对性服务重启被沙盒拒绝。未尝试绕过限制、未删除套接字、未终止 Chrome 或主应用、未修改浏览器数据、TCC 或签名。
下一步由用户正常退出并重新打开 ChatGPT 桌面应用，保留 Chrome，再在本任务验证控制连接。重启是否能恢复尚未验证。

## 数据和训练状态

未提交新训练，未替换运行模型。保留 status.json 中记录的未完成表单，恢复后必须核对宽高再提交。
本次对既有数据清单重新计算 SHA-256：152 个文件，变化 0，缺失 0。这验证的是本地清单；故障期间无法从网页复查云端数据。
