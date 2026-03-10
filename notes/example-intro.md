## 示例：如何组织我的机器人学习笔记

这是一篇示例笔记，用来说明你可以如何在这个站点中管理以前写过的文章。

### 1. 笔记来源

- 公众号文章：可以复制正文内容，稍微整理排版后粘贴到 Markdown 文件中。
- 语雀文档：语雀支持导出为 Markdown，将导出的 `.md` 直接放进 `notes/` 目录即可。

### 2. 文件放在哪里

建议约定：

- 所有 Markdown 笔记存放在仓库根目录下的 `notes/` 文件夹中。
- 文件命名尽量使用英文或下划线，避免空格，例如：
  - `notes/rl-pendulum-notes.md`
  - `notes/quadruped-controller-design.md`
  - `notes/wechat-some-article.md`

### 3. 在页面中“登记”这篇文章

打开 `notes.html`，在底部的 `notesConfig` 数组中添加一条记录，例如：

```js
{
  id: "rl-pendulum-notes",
  title: "强化学习倒立摆实验记录",
  path: "notes/rl-pendulum-notes.md",
  tags: ["rl", "pendulum", "control"],
  source: "Yuque",
}
```

保存并部署之后，这篇笔记就会出现在左侧列表中，可以搜索和点击查看了。

### 4. 与语雀的显示尽量一致

为了尽量接近语雀的阅读体验：

- 标题使用 `#`、`##`、`###` 等 Markdown 标记。
- 使用有序 / 无序列表来表达关键点。
- 使用代码块展示公式推导中的伪代码或命令行。

例如，简单的伪代码：

```python
for episode in range(num_episodes):
    state = env.reset()
    done = False
    while not done:
        action = policy(state)
        next_state, reward, done, _ = env.step(action)
        update_policy(state, action, reward, next_state)
        state = next_state
```

你可以删除或修改这篇示例笔记，只保留你自己的内容。

