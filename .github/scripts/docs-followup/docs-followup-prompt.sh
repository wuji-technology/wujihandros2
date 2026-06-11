#!/usr/bin/env bash
# docs-followup-prompt.sh
#
# Render the docs follow-up Claude prompt for a merged code PR.
# Self-hosted variant for wujihandros2 — hardcoded product config
# (Wuji Hand ROS2 / ros2-user-guide section). No dependency on
# wuji-docs-center repos.json.
#
# Required environment variables:
#   REPO         owner/name of the repository
#   PR_NUMBER    The merged code PR number
#   PR_TITLE     The merged code PR title
#   PR_AUTHOR    GitHub login of the merged code PR author

set -euo pipefail

: "${REPO:?REPO env required (owner/name)}"
: "${PR_NUMBER:?PR_NUMBER env required}"
: "${PR_TITLE:?PR_TITLE env required}"
: "${PR_AUTHOR:?PR_AUTHOR env required}"

PRODUCT_NAME="Wuji Hand ROS2"
DOCS_PATH="docs/external"
SURFACE_HINT="ROS2 节点话题/服务/参数、launch 文件接口、msg/srv 定义、面向用户的配置项"
NOISE_HINT="内部节点重构、CMake 微调、单测；不影响话题契约的实现细节"
EXTRA_PATHS="\`launch/\`、\`config/\`、\`README.md\`"
CHANGELOG_HINT="话题/参数变更需标明兼容性影响；launch 参数改名属于破坏性变更"

# Defensive: GitHub PR titles can in principle contain triple backticks, which
# would prematurely close the Feishu fenced code block this prompt renders into.
PR_TITLE_SAFE="${PR_TITLE//\`\`\`/ʼʼʼ}"

cat <<EOF
你是 ${PRODUCT_NAME} 文档维护助手。仓库 ${REPO} 的 PR #${PR_NUMBER}（${PR_TITLE_SAFE}，作者 @${PR_AUTHOR}）已合入 main。

任务：
1. 取代码改动：运行 \`gh pr diff ${PR_NUMBER} -R ${REPO}\`，并读 PR 描述
2. 读取本仓库对外文档目录的现有内容（\`${DOCS_PATH}/\`、${EXTRA_PATHS}）
3. 判断这次改动是否产生用户可感知变化：${SURFACE_HINT}
4. 需要更新 → 修改对应文档文件；文档缺失而变更需要新文档 → 新建
5. 不产生用户可感知变化 → 回复"无需改动"并说明理由

判断规则：
- 只关注用户可感知变更（${SURFACE_HINT}）
- 忽略 ${NOISE_HINT}

写作规范：
- 遵守 Microsoft Writing Style Guide
- 中文：删除"的/了/进行"等冗余词；用阿拉伯数字；主动语态、现在时
- 同一概念始终用同一术语；段落 3-7 行
- 不引入 frontmatter description 字段
- ${CHANGELOG_HINT}
EOF
