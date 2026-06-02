# Jetson Nano 热点与 SSH 连接

Jetson Nano 可以通过网线、现有 Wi-Fi 局域网或 Nano 自建热点被电脑访问。SSH 连接时最关键的一点是：目标 IP 必须是 Nano 这台机载电脑自己的本地 IP，而不是电脑自己的 IP，也不是截图里的示例 IP。

本项目推荐在需要外场直连时使用 Nano 热点。常用配置如下：

```text
热点名称 / SSID: auto-drone-nano
热点密码: 00000000
Nano 热点 IP: 192.168.1.100 或 192.168.0.100
子网掩码: 255.255.255.0
前缀长度: 24
SSH 示例: ssh jetson@192.168.1.100
```

实际使用时，以 `ifconfig` 或 `ip addr` 查到的 Nano 网卡 IP 为准。

## 1. 先用 ifconfig 确认 Nano 的本地 IP

在 Jetson Nano 上打开终端，先安装并运行 `ifconfig`：

```bash
sudo apt update
sudo apt install -y net-tools
ifconfig
```

也可以使用系统自带的 `ip addr`：

```bash
ip addr
```

查看输出时忽略 `lo`，它是本机回环地址，通常显示 `127.0.0.1`，不能用于电脑 SSH 连接。重点看 `eth0`、`wlan0`、`wlp...`、`wlx...` 这类网卡中的 `inet` 字段。

典型输出如下：

```text
eth0: ...
    inet 192.168.1.100  netmask 255.255.255.0 ...

wlan0: ...
    inet 192.168.0.100  netmask 255.255.255.0 ...
```

这里的 `192.168.1.100` 或 `192.168.0.100` 就是 Nano 的本地 IP。电脑要 SSH 到 Nano，就填这个 IP：

```bash
ssh jetson@192.168.1.100
```

如果 Nano 插了网线，`eth0` 就会显示一个稳定的公网 IP，例如 `101.6.163.107`，此时可以直接 SSH，不需要创建并链接 Nano 的热点。

如果 Nano 没有插网线，就需要创建热点然后连接。

本地地址一般长得像 `192.168.x.x`，例如：

```text
Nano IP: 192.168.1.100
电脑 IP: 192.168.1.101
```

前三段相同表示在同一个网段。子网掩码 `255.255.255.0` 或前缀 `/24` 都表示这个规则。

## 2. 根据本地地址前缀配置 Nano 热点

创建热点时，需要先选定 Nano 热点 IP。推荐选一个容易记的地址：

```text
Nano 热点 IP: 192.168.1.100
子网掩码: 255.255.255.0

或

Nano 热点 IP: 192.168.0.100
子网掩码: 255.255.255.0
```

选定后，后续所有 `ping`、`ssh`、`scp`、`rsync` 命令里的 IP 都使用 Nano 热点 IP。例如选 `192.168.0.100`，就写 `ssh jetson@192.168.0.100`。

### 2.1 进入网络连接管理界面

点击 Jetson Nano 桌面右上角网络图标，进入 `Edit Connections` / `编辑连接`。

![打开桌面右上角网络菜单](/images/nano-hotspot/nano-hotspot-01.png)

进入 Network Connections 后，可以看到已有的网线连接和 Wi-Fi 连接。点击左下角加号新建连接。

![进入 Network Connections 连接管理窗口](/images/nano-hotspot/nano-hotspot-02.png)

选择连接类型为 `Wi-Fi`，然后点击创建。

![新建连接并选择 Wi-Fi 类型](/images/nano-hotspot/nano-hotspot-03.png)

### 2.2 设置 Wi-Fi 名称和热点模式

在 `Wi-Fi` 标签页中填写：

```text
Connection name: auto-drone-hotspot
SSID: auto-drone-nano
Mode: Hotspot
Band: Automatic 或 bg
```

`SSID` 就是电脑 Wi-Fi 列表中显示的无线名称。这里填 `auto-drone-nano`，电脑之后就连接名为 `auto-drone-nano` 的 Wi-Fi。

> 强烈建议选择个性化的 SSID，避免和其他 Wi-Fi 冲突。

![在 Wi-Fi 标签页设置连接名、SSID 和 Hotspot 模式](/images/nano-hotspot/nano-hotspot-04.png)

### 2.3 设置热点密码

进入 `Wi-Fi Security` 标签页：

```text
Security: WPA & WPA2 Personal
Password: 00000000
```

`Password` 就是电脑连接这个热点时输入的 Wi-Fi 密码。公开使用或长期使用时应改成更安全的密码。

![在 Wi-Fi Security 标签页设置 WPA/WPA2 密码](/images/nano-hotspot/nano-hotspot-05.png)

### 2.4 设置 Nano 热点 IP

进入 `IPv4 Settings` 标签页。这里填写的是 Nano 这台机载电脑在热点网络里的地址。

如果选择 `192.168.1.100` 作为 Nano 热点 IP：

```text
Method: Shared to other computers
Address: 192.168.1.100
Netmask: 255.255.255.0
Gateway: 192.168.1.1
DNS servers: 留空
```

SSH 时连接的是 `Address` 这一栏，不是 `Gateway` 这一栏。也就是说，`Address` 填 `192.168.1.100`，SSH 就是 `ssh jetson@192.168.1.100`。

![在 IPv4 Settings 标签页设置 shared 模式和热点地址](/images/nano-hotspot/nano-hotspot-06.png)

### 2.5 关闭 IPv6 并保存

进入 `IPv6 Settings` 标签页，将 Method 设为 `Ignore` 或 `Disabled`，然后保存连接。

![在 IPv6 Settings 标签页关闭 IPv6](/images/nano-hotspot/nano-hotspot-07.png)

保存后，连接列表中会出现刚创建的热点连接。

![保存后的热点连接](/images/nano-hotspot/nano-hotspot-08.png)

### 2.6 启动热点并让电脑连接

创建热点后，需要让 Nano 切换到热点模式。

在 Nano 桌面右上角网络菜单中选择 Connect to Hidden Wi-Fi Network 选项

![在 Wi-Fi 连接列表中选择热点连接](/images/nano-hotspot/nano-hotspot-10.png)

点击选择连接名，对应的 WiFi 名即为外部连接此 Nano 时选择的 WiFi 名

![必要时输入热点连接密码](/images/nano-hotspot/nano-hotspot-11.png)

弹出下面的提示框则成功切换到热点模式

![热点连接建立成功](/images/nano-hotspot/nano-hotspot-12.png)

此时在个人电脑上打开 Wi-Fi 列表，连接：

```text
Wi-Fi 名称 / SSID: auto-drone-nano
Wi-Fi 密码: 00000000
```

电脑连接 Nano 热点后，通常会失去原有 Wi-Fi 的互联网访问，这是正常现象。此时电脑和 Nano 处在同一个小局域网里。

## 3. 设置热点开机自启

图形界面配置完成后，用 `nmcli` 确认连接名：

```bash
nmcli connection show
```

假设热点连接名为 `auto-drone-hotspot`，执行：

```bash
sudo nmcli connection modify auto-drone-hotspot connection.autoconnect yes
sudo nmcli connection modify auto-drone-hotspot connection.autoconnect-priority 100
sudo nmcli connection modify auto-drone-hotspot 802-11-wireless.mode ap
sudo nmcli connection modify auto-drone-hotspot ipv4.method shared
sudo nmcli connection modify auto-drone-hotspot ipv6.method ignore
```

如果 Nano 热点 IP 设为 `192.168.1.100`：

```bash
sudo nmcli connection modify auto-drone-hotspot ipv4.addresses 192.168.1.100/24
```

重新启动热点：

```bash
sudo nmcli connection down auto-drone-hotspot || true
sudo nmcli connection up auto-drone-hotspot
```

检查自启配置：

```bash
nmcli connection show auto-drone-hotspot | grep -E 'autoconnect|ipv4.addresses|802-11-wireless.mode'
```

重启 Nano 后，如果电脑能再次看到 `auto-drone-nano` 热点，并且 `ifconfig` 里无线网卡仍显示设定的 `192.168.x.100`，说明开机自启成功。

## 4. SSH 连接与常用 Linux 命令

先确认 Nano 上 SSH 服务已安装并启动：

```bash
sudo apt install -y openssh-server
sudo systemctl enable --now ssh
sudo systemctl status ssh
```

状态中出现 `active (running)` 即表示 SSH 服务正常。

在电脑终端连接 Nano。IP 填 `ifconfig` 查到的 Nano 本地 IP：

```bash
ssh jetson@192.168.1.100
```

首次连接会提示是否信任主机指纹，输入 `yes`。随后输入 Nano 本机 `jetson` 用户的登录密码。成功后，终端提示符会切换到 Nano。

常用命令：

```bash
pwd                         # 查看当前目录
ls                          # 查看当前目录文件
ls -la                      # 查看隐藏文件和详细信息
cd                          # 进入指定目录
mkdir                       # 新建目录
cp a.txt b.txt              # 复制文件
mv old.txt new.txt          # 重命名或移动文件
rm file.txt                 # 删除文件
cat file.txt                # 查看短文本文件
ip addr                     # 查看网卡和 IP
ifconfig                    # 查看网卡和 IP
df -h                       # 查看磁盘空间
free -h                     # 查看内存
exit                        # 退出 SSH
```

从电脑同步代码到 Nano，推荐使用 `rsync`：

```bash
rsync -av --delete \
  --exclude .git \
  --exclude node_modules \
  --exclude docs/.vitepress/dist \
  auto-drone/ \
  jetson@192.168.1.100:~/auto-drone/
```

也可以使用 `scp`：

```bash
scp -r auto-drone jetson@192.168.1.100:~/
```

连接 Nano 后运行项目命令：

```bash
cd ~/auto-drone/flight
/usr/bin/python3 src/connect_pixhawk.py --system-address serial:///dev/ttyTHS1:921600
/usr/bin/python3 src/rc_mode_monitor.py --device /dev/ttyTHS1 --baud 921600
```

自主前飞：

```bash
cd ~/auto-drone/flight
./scripts/run_guided_forward.sh 0.8 1.5 0.2
```

视觉跟随：

```bash
cd ~/auto-drone/vision
./scripts/run_gate_follow.sh
```

## 5. 常见问题

电脑找不到热点：

```text
确认 Nano 已经开机完成
确认热点连接已经启动：sudo nmcli connection up auto-drone-hotspot
确认无线网卡支持 AP / Hotspot 模式
尝试重启 NetworkManager：sudo systemctl restart NetworkManager
```

电脑能连接热点但 ping 不通：

```text
在 Nano 上用 ifconfig 查看实际 IP
电脑 ping 的 IP 必须是 Nano 的 inet 地址
电脑 IP 和 Nano IP 前三段要一致，例如同为 192.168.1.x
子网掩码填 255.255.255.0
如果电脑手动配置，网关填 Nano IP
```

SSH 连接被拒绝：

```text
确认 sudo systemctl status ssh 显示 active
确认用户名正确，例如 jetson
确认 SSH 目标 IP 是 Nano IP，不是电脑 IP
确认 Nano 本机账户密码正确
```

重启后热点没有自动启动：

```bash
nmcli connection show
nmcli connection show auto-drone-hotspot | grep autoconnect
sudo nmcli connection modify auto-drone-hotspot connection.autoconnect yes
sudo nmcli connection up auto-drone-hotspot
```

连接热点后电脑无法上网：

```text
这是正常现象。电脑当前 Wi-Fi 被 Nano 热点占用，只能访问 Nano 局域网。
需要互联网时，临时切回原 Wi-Fi，或给 Nano 配置其他联网方式。
```

## 参考

- Jetson Nano 热点图形界面配置流程参考：[Jetson Nano 创建热点教程](https://blog.csdn.net/qq_35598561/article/details/140079206)
- 本页图形界面截图整理自上述教程页面，命令行配置和项目默认网络参数按 `auto-drone` 项目实际使用流程编写。
