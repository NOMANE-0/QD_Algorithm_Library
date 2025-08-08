# 奇点算法组知识库

直接访问 <https://nomane-0.github.io/QD_Algorithm_Library/#/>

## 本地使用指南

- 使用cmd输入命令看有没有输出版本号，没有需要去安装`Node.js`和`npm`

```bash
node -v
npm -v
```

- 安装

```bash
npm i docsify-cli -g
```

- 检验安装

```bash
docsify -v
```

- 在根目录下输入以下命令启动网页

```bash
docsify serve docs
```

- 访问 <http://localhost:3000>

## 编写指南

知识库分成多个模块，每个模块有自己的文件夹，每个文件夹里都有`images`文件夹存放该模块使用的图片

### 编写内容

修改原有内容的话直接修改就可以生效了，但需要注意，复制粘贴要保证图片指向修改文件所在目录下的`images`文件夹里

想添加新文件的话只需要正常在模块下写`.md`就可以，然后在所在目录的`_sidebar.md`里添加索引

> 根目录下的`.vscode`里设置了复制图片会默认把图片放到所在目录的`images`里，如果你使用 vscode 则不需要担心图片路径问题

> 不要往知识库塞大文件，像直接把其他开源的代码直接塞进来，尽量采用引用的方式

### 添加模块

想添加一个新模块只需要在[_navbar.md](docs\_navbar.md)中仿照里面的内容修改

在里面添加如下内容，模块名是你自定义的，`path`是相对于`docs`路径的，例如`- [其他](/Others/)`就是模块名为其他，路径为`/docs/Others/`

```terminal
- [模块名](/path/)
```

创建一个新模块后需要在相应的文件夹里创建一个有内容的`README.md`作为默认显示，并创建一个`_sidebar.md`作为侧边栏目录

## 提交修改

`fork`该仓库然后提交`pull requests`
