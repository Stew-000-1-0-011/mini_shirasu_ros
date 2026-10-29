# mini_shirasu_ros

mini-shirasu (ブラシ付き DC モータドライバ。ファームは
[mini-shirasu2](https://github.com/Stew-000-1-0-011/mini-shirasu2) の minishirasu-firm) と、
[robomas_plugins](https://github.com/Stew-000-1-0-011/robomas_plugins) の USB-CAN ブリッジ
(`robomas_can_tx` / `robomas_can_rx`) 越しに話す ROS 2 パッケージ。

プロトコルは mini-shirasu2 の `docs/superpowers/specs/2026-10-03-minishirasu-firm-design.md`
の「プロトコル」「CAN」と、ファームの実装 (`minishirasu-firm/src/protocol.rs`, `state.rs`, `config.rs`) に合わせてある。
**CAN 越しに実機の基板と話す確認はまだしていない** (確認は同梱の模擬基板 `fake_mini_shirasu` で行った)。

対応環境: ROS 2 Lyrical Luth / Ubuntu 26.04、C++26 (`CMAKE_CXX_STANDARD` で上書き可。C++23 以降が必須)。

## 構成

| ファイル | 役割 |
| --- | --- |
| `include/mini_shirasu_ros/protocol.hpp`, `src/protocol.cpp` | メッセージ、COBS、CRC-8/SMBUS、ストリームパーサ、CAN フレームへの分割。ROS非依存 |
| `include/mini_shirasu_ros/settings.hpp` | SetParam の設定 ID と名前の対応 |
| `src/mini_shirasu_node.cpp` | ノード本体 (基板 1 枚ぶん)。ROSの入出力と手順だけを見る |
| `msg/Status.msg` | 基板の状態 (物理単位) |
| `launch/mini_shirasu_node.launch.py` | 起動。`bridge:=true` で robomas_bridge も立てる |
| `test/protocol_test.cpp` | プロトコルのテスト (ROS不要) |
| `test/fake_mini_shirasu.cpp` | 模擬基板。CAN のトピックを受けて、ファームの状態機械を真似て返す |


**パラメータに既定値は無い。** 全部パラメータファイルで与える (足りなければ起動時にエラーで止まる)。この機体の値と各項目の説明は oumuamua の `src/oumuamua_bringup/config/mini_shirasu.yaml` にある。

## 使い方

```bash
ros2 launch mini_shirasu_ros mini_shirasu_node.launch.py params:=/path/to/mini_shirasu_node.yaml bridge:=true
ros2 topic pub -r 20 /mini_shirasu_node/target_velocity std_msgs/msg/Float64 "{data: 50.0}"
```

### トピックとサービス

| 方向 | 名前 | 型 |
| --- | --- | --- |
| pub | `can_tx_topic` (例 `robomas_can_tx`) | `robomas_plugins/msg/Frame` |
| sub | `can_rx_topic` (例 `robomas_can_rx`) | `robomas_plugins/msg/Frame` |
| sub | `~/target_current` | `std_msgs/msg/Float64` [A] (`mode: current` のとき) |
| sub | `~/target_velocity` | `std_msgs/msg/Float64` [rad/s] (`mode: velocity` のとき) |
| sub | `~/target_position` | `std_msgs/msg/Float64` [回転] (`mode: position` のとき) |
| sub | `~/target_duty` | `std_msgs/msg/Float64` [-1, 1] (`mode: duty` のとき。電流ループを使わずそのまま PWM に出す。ファームの `duty_max` で切り詰められる。デューティモードの入ったファーム (mini-shirasu2 5617592 以降) が要る) |
| pub | `~/status` | `mini_shirasu_ros/msg/Status` |
| pub | `~/joint_state` | `sensor_msgs/msg/JointState` (Status の位置 [rad]・速度 [rad/s]・電流 [A] を標準の型で。車輪オドメトリ用) |
| srv | `~/enable` | `std_srvs/srv/SetBool` (true で `mode` に、false で無効に) |
| srv | `~/reset_fault` | `std_srvs/srv/Trigger` (異常ラッチの解除。設定前なら解除してから設定する) |
| srv | `~/set_origin` | `std_srvs/srv/Trigger` (いったん無効にして原点を取り、元に戻す) |
| srv | `~/reconfigure` | `std_srvs/srv/Trigger` (設定をやり直す) |

サービスはコマンドを積むだけで、結果はログに出る (基板の応答は非同期なので)。

### 起動時の手順

1. `reset_fault_on_start: true` なら `ResetFault` (前回の異常ラッチが残っていても設定できるように。既定は false)
2. `SetMode(無効)` (設定は無効中しか受け付けないので)
3. `settings.*` を設定 ID の順に `SetParam`
4. `enable_on_start: true` なら `SetMode(mode)`

コマンドは 1 つずつ送り、Ack を待ってから次を送る。`command_timeout` で応答が無ければ
`command_retries` 回まで再送する。

- 設定中に応答が無いまま再送が尽きたら、基板かブリッジがまだ起きていないとみなし、1 秒後に最初からやり直す
- Nack が返ったら理由 (設定エラーなら原因の設定名も) をログに出し、残りの手順は送らない。
  ただし `SetParam` への設定エラーは、最後の `SetParam` までは無視して続ける
  (基板は値を保存してから設定全体を検査するので、前回の無効な値が残っていると途中で返る)
- 設定が済んだ後に Status が `status_timeout` 以上途切れたら、基板が再起動して設定が消えたとみなし、設定し直す
  (`status_period_ms` を 0 にするなら `status_timeout` も 0 にすること)

### 目標値

`control_rate` の周期で、`mode` に合った目標値を送り続ける。
**ファームには通信タイムアウトが無い**ので、目標が `target_timeout` 以上来なければ、
電流・速度モードではゼロを送る。位置モードではゼロに戻すと動いてしまうので、何も送らない
(ファームは最後の目標を保つ)。

加速度 FF・速度 FF は今は送らない (0)。

## 設定 (`settings.*`)

名前と単位は minishirasu-firm の設計書「設定値」と同じ。`status_period_ms` 以外はすべて必須で、
1 つでも欠けていればノードは起動しない。値は f32 で送られる。真偽値 (`encoder_reversed`) は 0 / 1。

**速度と位置は、`encoder_cpr` が 1 回転と数える軸が基準**になる。エンコーダの付いた軸ではなく、
`encoder_cpr` に減速比を織り込めば駆動軸 (ホイール) 基準にできる。

oumuamua の `src/oumuamua_bringup/config/mini_shirasu.yaml` の `settings` は、mini-shirasu2 の `minishirasu-firm/src/bench.rs` で
実機 (RZ-735VA-8519、減速比 11.86) の電流・速度・位置モードが動いた値で、駆動軸基準。
別のモータで使うなら bench.rs のコメントを見て決め直すこと。

## CAN ID

ストリームごとに ID を 1 つ使う。ファームの `config.rs` のコンパイル時定数と合わせること。

| パラメータ | 例 (oumuamua の wheel0) | 向き |
| --- | --- | --- |
| `can_id.target` | 0x110 | 送信 (目標値) |
| `can_id.status` | 0x120 | 受信 (状態) |
| `can_id.command` | 0x130 | 送信 (コマンド) |
| `can_id.response` | 0x140 | 受信 (応答) |

複数枚つなぐときは基板ごとに ID を変え、ノードも基板ごとに 1 つ立てる。

## 注意

- **CAN は 1Mbps、標準 ID**。ビットレートは USB-CAN ブリッジ (robomas_plugins が話す Debug_CAN ボード)
  側で合わせること。このノードは ID とデータしか扱わない
- **ファームには通信タイムアウトが無い**。このノードは目標が途切れたらゼロを送り、
  正常終了時には `SetMode(無効)` を送るが、ノードやブリッジ、PC が落ちたときは出力が止まらない。
  止めるのは緊急停止スイッチに頼ること
- メッセージは CAN フレームをまたぐ (SetParam は 9 バイトで 2 フレーム、Status は 24 バイトで 3 フレーム)。
  ID ごとに連結して 0x00 で切り出している

### 応答が来ないとき

ファームの CAN 送受信 (CanRx / CanTx) はまだ実機で確かめられていない。応答が来ないときは、どこで止まっているかを順に見る。

1. `ros2 topic echo /robomas_can_tx` で、コマンド (ID 0x130) のフレームが出ていること
2. robomas_bridge のログに `negotiation success` が出ていること (出るまでブリッジは何も送らない)
3. `ros2 topic echo /robomas_can_rx` で、応答 (ID 0x140) や Status (ID 0x120) のフレームが返ってきているか
4. 基板のログ (defmt / RTT) に `nack:` の行が出ていないか。出ていれば理由と設定 ID が分かる

3 で何も返ってこず、基板のログにも何も出ないなら、ファーム側の受信を疑う。
そのときは基板のログを mini-shirasu2 側に渡す。

## テスト

```bash
colcon test --packages-select mini_shirasu_ros && colcon test-result --verbose
```

`protocol_test` は minishirasu-firm の `protocol.rs` のテストと同じ性質を見る
(CRC の検査値、全メッセージの往復、符号化後の長さと種別、途中からの受信・化け・あふれからの復帰、
CAN フレームへの分割と再結合、Q16.16)。

模擬基板でノードを動かすには:

```bash
ros2 run mini_shirasu_ros fake_mini_shirasu
ros2 run mini_shirasu_ros mini_shirasu_node --ros-args -r __node:=wheel0 --params-file /path/to/oumuamua/src/oumuamua_bringup/config/mini_shirasu.yaml
```
