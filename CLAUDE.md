# 项目开发规范

## PR Review 评论回复 SOP

当 PR 收到 CodeRabbit 或其他 reviewer 的评论时，按以下流程处理：

### 1. 获取 PR 评论

```bash
# 获取 PR 的普通评论和 review 信息
gh pr view <PR号> --comments --json comments,reviews

# 获取代码行级别的 review comments（包含评论 ID）
gh api repos/<owner>/<repo>/pulls/<PR号>/comments
```

### 2. 评论类型说明

| 类型 | 说明 | 位置 |
|-----|------|------|
| Issue Comment | PR 页面底部的普通评论 | PR 讨论区 |
| Review Comment | 代码行级别的审查评论 | 代码 diff 中 |

### 3. 回复评论

**回复普通评论：**
```bash
gh pr comment <PR号> --body "回复内容"
```

**回复代码行级 Review Comment（需要评论 ID）：**
```bash
gh api repos/<owner>/<repo>/pulls/<PR号>/comments \
  -X POST \
  -f body="回复内容" \
  -F in_reply_to=<comment_id>
```

### 4. 关键字段说明

从 `gh api .../pulls/<PR号>/comments` 返回的 JSON 中：

| 字段 | 说明 |
|-----|------|
| `id` | 评论 ID，用于回复时的 `in_reply_to` 参数 |
| `body` | 评论内容 |
| `path` | 评论所在文件路径 |
| `line` | 评论所在行号 |
| `user.login` | 评论者用户名 |

### 5. 完整操作示例

```bash
# 步骤 1: 获取 review comments，找到需要回复的评论 ID
gh api repos/wuji-technology/wujihandros2/pulls/16/comments

# 步骤 2: 回复指定评论（假设评论 ID 为 2684533167）
gh api repos/wuji-technology/wujihandros2/pulls/16/comments \
  -X POST \
  -f body="已修复。" \
  -F in_reply_to=2684533167
```

### 6. 处理流程

1. **获取评论** - 使用上述命令获取所有待处理评论
2. **分析问题** - 理解每个评论指出的问题
3. **修复代码** - 根据建议修改代码
4. **提交更新** - `git commit --amend` 或新提交
5. **推送更新** - `git push --force-with-lease`
6. **回复评论** - 使用 API 回复每个评论，说明已修复
