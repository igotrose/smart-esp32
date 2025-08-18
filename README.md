# HelloWorld 快速开始
## 构建工程
直接从`esp-idf`的`example`中直接复制`helloworld`作为初始化工程
- 创建工程
```bash 
mkdir -p smart-esp32s3
cp ../esp-idf/examples/helloworld .
```
- 初始化仓库
```bash
git init
git checkout -b main 
git add .
git commit -m "helloworld"
git checkout -b helloworld
```
- 设置环境
```bash
. ../esp-idf/export.sh
idf.py set-target esp32s3
```
- 编译烧录监视
```bash
# 编译
idf.py build 
# 烧录
idf.py -p /dev/ttyUSB0 flash
# 监视
idf.py -p /dev/ttyUSB0 monitor
# 退出监视
ctrl + ]
```
- 合并分支
```bash
git checkout main 
git merge helloworld
```