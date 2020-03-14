# USBJOY_HUYE

@huye_4589 さんが作成されているJOYDRV3関連のファイルです。
本家ページは http://park7.wakwak.com/~huye/kaihatsu/ になります。

## usbjoy_to_atari.ino
USBJOYは、arduino + USB HOST Shield にてUSB接続したコントローラをATARI仕様のコントローラへ変換するarduinoのスケッチです。
D-SUB9ピンのATARIコントローラシールドはhuyeさんが専用基板を作成されていますが、自分で配線することも出来ます。
例えば、https://www.aitendo.com/product/18808 これがあれば半田付け作業なしに接続することも出来ます。

### 対応arduino
現在確認出来ているarduinoは以下の通りです。

* arduino uno (ATmega328P 16MHz 5V)
* arduino pro mini (ATmega328P 16MHz 5V)
arduino正規品及び互換品にて稼働確認をしています。

* https://www.amazon.co.jp/dp/B0168B39N4/ref=cm_sw_r_tw_dp_U_x_NGpBEbTA4ATH9
* https://www.amazon.co.jp/dp/B01CX2AQEW/ref=cm_sw_r_tw_dp_U_x_ZDpBEbG9EBA4E

pro miniのATmega168ではテストを行っておりません。
pro miniは直接USBを接続しての書き込みが出来ないため、別途FT232RL等が必要になります。

### 対応USB HOST Shield
arduino対応の5Vが扱えるものであれば大丈夫そうです。
aitendoで売ってたものを使用していますが、今は扱ってない模様。
また arduino pro mini用のUSB HOST Shieldは、http://www.takishita.jp/toy_hospital/sale/arduino_pro_mini_usb_host_shield で稼働しています。
pro miniは 8MHzや3.3Vなどバリエーションがあるので、16MHz 5Vの物を選ぶ必要があります。

### 対応コントローラ
Ver1.20から対応コントローラが増えました（私が持っているものを組み込ませて頂きました）
特にX68000でアケコンを使いたい願望があり、HORIのRAPシリーズ(PS3/PS4)に対応しています。
他のメーカーのアケコンでも流れているデータがわかれば追加することが出来ます。

またXBOXはPS3/PS4とスケッチの共存することが出来ないためXBOX用のスケッチを使用することになります。

## JOYDRV3

### KONAMI.DAT
### CAPCOM.DAT
### GAROSP.DAT
