# 记住内核启动顺序

- 编辑`grub`文件

```terminal
sudo vim /etc/default/grub

#修改 GRUB_DEFAULT=0 ，
GRUB_DEFAULT=saved
#加入
GRUB_SAVEDEFAULT=true
```

- 更新`grub`

```terminal
sudo update-grub
```
