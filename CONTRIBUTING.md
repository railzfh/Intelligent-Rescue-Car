# 两人协作与 GitHub 启用指南

本项目计划使用 **公开仓库**，默认分支为 `main`。当前协作者之一为 `railzfh`；另一位的用户名待补充。
项目文件夹叫 `Intelligent-Rescue-Car`，固件工程名暂保留 `dm02_test`。
所有智能体先读 [AGENTS.md](AGENTS.md)，规则通过正常 PR 修订。

## 1. 三层约束

| 层级 | 负责什么 | 生效条件 |
| --- | --- | --- |
| AGENTS.md 及工具入口 | 保留他人工作、任务分支、真实验证、真人审阅 | 智能体读取并遵守 |
| 本地 pre-push | 拦截集成分支/标签推送、远程删除、非快进更新、违规文件 | 每个 clone/worktree 安装 hooks |
| GitHub Rulesets | 拦截强推、保护集成分支、要求另一人批准及 CI 通过 | 仓库管理员在服务器启用 |

文件放进仓库不会自动启用 GitHub 规则。本地 hook 可被绕过，不能替代服务器端保护。
规则集也无法证明业务逻辑正确；编译与真人审阅仍然必要。

## 2. 首次公开建仓（由真人负责人执行）

1. 在 GitHub 创建公开的 `Intelligent-Rescue-Car` 空仓库，不自动生成 README、LICENSE 或 `.gitignore`。
   所有者由两位协作者确定，不能从本机用户名推断。
2. 首次上传前检查待提交内容，特别是原始源码、资料包和第三方许可证。
   保留 ST/CMSIS 等第三方版权声明；尚未选择项目自身许可证，不擅自替第三方资料授权。
3. `资料.7z` 是用户指定必须上传的文件，大小约 192 MB，超过普通 Git 的 100 MiB 单文件限制，
   已在 `.gitattributes` 中指定 Git LFS。首次暂存前，初始化 Git 和 LFS 过滤器：

   ```powershell
   git init -b main
   git lfs install --local --skip-repo
   git add -- .gitattributes
   git add -- 资料.7z
   git lfs ls-files
   ```

   然后按明确路径添加源码和规则文件，使用 `git status --short`、`git diff --cached --stat`
   和 `python scripts/check_repository.py --staged` 检查。不要直接把压缩包当普通 blob 提交。
   LFS 使用账户存储/下载额度，后续更新会增加对象；CI 不下载资料包。
4. 资料包不因含本机状态就被自动删除或重打包。需要检查包内内容；压缩包有嵌套包时，外层文件名检查
   不能代替嵌套内容检查。发现问题先修复再公开；只报告位置，不在日志中展示凭据。
5. 真人完成本地首个提交，关联双方确认的仓库 URL。首个 `main` 是建仓唯一引导操作，
   在它之后立即启用下述保护，不能把这个引导例外用于日常开发。首次推送前安装 Git LFS 标准 hook
   （`git lfs install --local`），确保资料对象上传；完成引导后再安装本项目组合 hook。
   智能体不自动提交/发布这个初始公开快照。
6. 先让 `main` 的 CI 完整运行一次；设置中仅允许 Squash merge，关闭自动合并。
   将另一位真实协作者加入仓库，核实其用户名与写入权限。
7. 在 **Settings → Rules → Rulesets → New ruleset → Import a ruleset** 分别导入：
   - `.github/rulesets/no-force-push.json`：全部分支禁止非快进更新。
   - `.github/rulesets/protect-main.json`：保护默认分支、main/master/develop/release 系列。
8. 确认两条规则都为 **Active**，bypass 列表为空；在 GitHub 页面核对以下配置：
   - 禁止删除集成分支；必须通过 PR；仅允许 Squash 合并。
   - 需要 **1 位其他协作者批准**，新提交撤销旧批准，最近一次推送需另一人批准。
   - 所有审阅讨论解决；目标分支保持最新；要求 `Repository policy` 与 `Firmware build` 成功。
   - 两个检查应来自 **GitHub Actions**。JSON 使用 GitHub.com Actions 的 app ID `15368`，导入后核对来源。
9. Rulesets 空 bypass 并不意味着管理员无法编辑规则。管理员凭据留给真人设置环境；
   智能体使用仅针对本仓库的内容/PR 工作权限，不授予 Administration 或规则绕过权限。
   两人分别用各自账号，不互用登录态；不能让提交者的智能体用审阅人账号代签批准。
10. 在测试任务分支验证 CI 失败会阻止合并、无人批准不能合并、追加提交后需重新批准；
    核对集成分支删除和全部分支强推规则确实生效。不要靠破坏主分支做验收。

两个人不需要额外要求“两份批准”。目前未创建 CODEOWNERS，避免把唯一 owner 固定成 `railzfh`
而导致他自己的 PR 无人能满足 owner 审批。补齐两人用户名后可以按职责设置；当前的一份外部批准仍然必需。

## 3. 每位成员的一次性本地设置

安装 Git、Git LFS、Python 3.10+；clone 后先读规则。建仓负责人需先完成上一节引导。

```powershell
python scripts/install_hooks.py
git config --get core.hooksPath
git lfs ls-files
```

本项目的 pre-push 同时调用策略检查与 `git lfs pre-push`，不能只安装其中一半。
安装器只改当前仓库配置，不改全局配置；已有其他 hooksPath 或自定义 pre-push 时拒绝覆盖，
需要显式整合。标准 Git LFS pre-push 可替换为本项目包含 LFS 调用的组合钩子。
钩子依赖 Git 调用时的仓库根目录，并同时兼容 Git for Windows 的 sh 与 Linux/macOS。
初次 clone 没有自动安装项目钩子；新增 clone/worktree 要核实 hook 是否实际启用。

## 4. 日常任务流程

1. 查看工作区，确认未提交改动归属；在干净工作区从最新 `origin/main` 创建本人任务分支：

   ```powershell
   git fetch origin
   git switch -c feat/railzfh/m2006-can origin/main
   ```

   另一位使用自己的 GitHub 用户名。一个分支只做一个任务，不能两人共用同一任务分支。
   并行智能体各用独立 worktree/clone；同一个 `.ioc` 或生成文件安排一个任务负责修改。
2. 完成修改与验证后，按路径暂存并审查：

   ```powershell
   git add -- User_File/4_Task/tsk_config_and_callback.cpp
   git diff --cached
   python scripts/check_repository.py --staged
   git commit -m "feat(motor): add M2006 CAN control"
   git push -u origin feat/railzfh/m2006-can
   ```

3. 向 `main` 创建 PR，填写模板，请另一人检查最终 diff、CI 和硬件验证说明。
   需要合并最新主线时，`git fetch origin` 后 `git merge origin/main`，解决冲突并新增提交；不强推。
4. 对方真人批准后，由真人点击 Squash merge。追加修改后重新审阅；智能体建议不能替代批准。
   为避免两人审批互锁，PR 作者负责该分支的全部推送（包括同步主线和按审查意见修改）；
   审阅人通过评论建议修改，不直接向对方的 PR 分支推提交。
5. 合并出错时，在新的任务分支 `git revert <合并提交SHA>`，验证后再走 PR。
   若凭据已经公开，先撤销/轮换凭据，历史清理由仓库负责人单独协调，智能体不自动重写历史。
6. 已合并任务分支可由真人在 GitHub 清理。本地 hook 默认拦截所有远程删除和标签推送，
   发布版本标签与仓库维护交给真人；不能通过换 API 或跳过 hook 绕过。

## 5. 验证命令

保护工具的测试不需要硬件：

```powershell
python -m unittest discover -s tests -v
python scripts/check_repository.py --staged
```

固件使用 CMake 4.0+、Ninja、GNU Arm Embedded GCC/G++。将工具加入 PATH 后，在干净目录构建：

```powershell
cmake -S . -B build/verify -G Ninja -DCMAKE_BUILD_TYPE=Debug "-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake"
cmake --build build/verify --parallel 2
```

如果文件夹改名后旧 `build/Debug` 缓存仍引用旧路径，另选新的构建目录，不清理他人的文件。
CI 使用干净 checkout 的 Debug preset，执行真实配置、编译和链接；不会连接电机或证明硬件安全。
源码不变的规则/文档修改仍需通过 CI；固件的上板行为按 PR 内容另行验证。

## 6. 敏感信息与资料包

`.gitignore` 排除环境文件、私钥/证书、本机 IDE 状态及构建产物；`资料.7z` 明确例外，并通过 LFS 上传。
`.a` 是构建必需的 CMSIS-DSP 依赖，不能为清理二进制文件而删掉。

`check_repository.py` 检查暂存区/提交对象，拦截敏感路径、常见 GitHub/AWS 凭据和私钥格式、
违规大 blob、资料包非 LFS 提交，以及关键规则文件缺失。推送 hook 与 PR CI 还检查中间提交，
防止“先提交凭据，再在最后一个提交删除”漏检。它不是全面密钥扫描器，不展开 LFS/压缩包。
GitHub 端另启用仓库可用的 Secret scanning / Push protection；不将它当成本地扫描的替代品。

规则/CI 可以被有写入权限的人提出修改，因此审阅人要专门检查 `.github/`、`.githooks/`、`scripts/`、
各智能体入口、`.gitignore` 与 `.gitattributes` 的变化。任何本地文件都无法约束持有管理凭据且主动关闭规则的人。

## 官方参考

- [Rulesets 与套餐支持](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-rulesets/about-rulesets)
- [可用规则与审阅要求](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-rulesets/available-rules-for-rulesets)
- [GitHub 文件大小限制](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github)
- [Git LFS 指针与存储](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-git-large-file-storage)
