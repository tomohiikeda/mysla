# 自作SLAM mysla
### これは何？
SLAMを自分で作ってみようというプロジェクト。HWはアールティ社のRaspberryPi Mouseを使用。

### 何ができるの？
何もできません。

### 環境
* HW: RaspberryPi Mouse
* OS: Raspbian(Version???)
* ドライバ: 
* コントローラ: 

### 準備

#### ドライバインストール
sudo insmod RaspberryPiMouse/src/drivers/rtmouse.ko
chmod 777 /dev/rt*

#### DualShock4のペアリング
/dev/input/js0 が見えているか確認する。

### ビルド方法
```
cd mysla  
make
```

### 実行方法

* SLAM実行モード  
SLAMを実行するモードです。
```
output/Linux/Release/myslam slam <offline or online> <ディレクトリ名> <開始インデックス> <最終インデックス>
```

* SlamData保存モード  
LidarとPulse Counterのデータを取得し、ファイルに保存するモードです。ファイル名は、pt_xxxx.txt(xxxxは通し番号)に保存されます。Interval値の単位は秒。
```
output/Linux/Release/myslam save <directory名> <Interval(sec)>
```

* Scanモード  
Lidarを起動し、取得データをGnuplotでリアルタイム表示します。
```
output/Linux/Release/myslam scan_plot
```

* ラジコンモード  
DualShock4で操作することができるモードです。
```
output/Linux/Release/myslam remocon
```

* スキャンマッチングテストモード  
ScanMatchingのテストを行うモードです。指定されたディレクトリ内のSlamDataファイルが使用され、開始インデックスと最終インデックスの間でScan Matchingを連続的に実行します。
```
output/Linux/Release/myslam matching <directory名> <開始インデックス> <最終インデックス>
```

* Usage  
実装されているモード一覧を表示します。
```
output/Linux/Release/myslam usage
```


