# Windows_Fonts

补充黑体、宋体等在 windows 上的常用字体

1. 将windows下存放字体的地方`c/Windows/Fonts/`复制到 Linux 下`/usr/share/fonts/`
2. 修改字体权限

```bash
sudo chmod 644 /usr/share/fonts/your_new_fonts
```

3. 更新字体缓存

```bash
fc-cache -vfs
```
