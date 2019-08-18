# SlidingMode
<!--
$$
\dot{\omega}=\frac{R}{J}f-\frac{1}{J}u
$$
## function Brk_CalcTotalBrk(void){

1. オブザーバ/張力推定器
---
  1. 技術解説  
<img src="fig1.png" style="float:right;" >  
張力はセンサー等ではなくスプールの回転速度からオブザーバで推定する。右図がこの力学系のモデル図である。この系の数式張モデルは下式となる。  
&nbsp;&nbsp;&nbsp;<img src="https://latex.codecogs.com/gif.latex?\dot{\omega}=\frac{R}{J}f-\frac{1}{J}u" />  
変数<i>u</i>はスプールに作用させるブレーキトルクである。これは制御系から見ると既知である。  
張力<i>f</i>の立式は困難であるが、定数と仮定  
&nbsp;&nbsp;&nbsp;<img src="https://latex.codecogs.com/gif.latex?\dot{f}=0" />  
すればオブザーバで推定可能(可観測)である。  
次にマイコンでの実装を容易にするため、以下の変数変換を行う。これはトルクを角加速度の次元に変換するものである。  
&nbsp;&nbsp;&nbsp;<img src="https://latex.codecogs.com/gif.latex?\beta=\frac{R}{J}f" />  
またブレーキトルクは発電制動によって得られるため、コイル電流と回転速度に比例する。さらにモータはコアレス構造のためインダクタンスが低く、電気時定数は100&micro;秒のオーダである。従ってコイル電流は印加電圧、つまりPWMデューティと比例すると見なせる(オームの法則)。以上を考慮すると、ブレーキトルクとデューティは以下の比例関係にある。  
&nbsp;&nbsp;&nbsp;<img src="https://latex.codecogs.com/gif.latex?\frac{1}{J}u=b\cdot\omega\cdot duty" />  
<i>b</i>はブレーキ定数と呼ぶ。式の見かけ上、これはこの系の時定数の逆数となる。これはトルク定数とコイルインピーダンスなどから求められるが、最終的には較正が必要である。較正手段は後述する。  
以上を考慮した系の数式モデルは下式となる。  
&nbsp;&nbsp;&nbsp;<img src="https://latex.codecogs.com/gif.latex?\dot{\omega}=\beta-c\cdot\omega\cdot duty" />  
&nbsp;&nbsp;&nbsp;<img src="https://latex.codecogs.com/gif.latex?\dot{\beta}=0" />  
この系に対する、同一次元オブザーバは下式となる。ただしオブザーバの極配置は、実数部-&lambda;、虚数部０の重極とする。  
&nbsp;&nbsp;&nbsp;<img src="https://latex.codecogs.com/gif.latex?\dot{\hat{\omega}}=-2\lambda(\hat{\omega}-\omega)+\beta-c\cdot\omega\cdot duty" />  
&nbsp;&nbsp;&nbsp;<img src="https://latex.codecogs.com/gif.latex?\dot{\hat{\beta}}=-\lambda^2(\hat{\omega}-\omega)" />  
実装上、サンプリングは定周期ではなく、1回転ごととなる。よって上式を&Delta;&theta;にて離散化することでマイコンで解くモデルを得る。
  
  2. コード解説  
以下のコードは、先のオブザーバモデルの数値解析部である。CPUの制約(8ビット)から、固定小数点演算にて処理する必要があり、固定小数点設計ツール https://github.com/KazukiHiraizumi/FixPointDesigner を使って設計している。

  定数
<table>
<tr><th>variable<td>pi2<td>pole<td>b2pi</tr>
<tr><th>description<td>2&pi;<td>サンプリング時間<td>角速度<td>角速度(小数部切捨て)<td>&omega;推定誤差<td>極<td><i>b&times;2&pi;</i><td>b補正(後述)</tr>
<tr><th>bit width</tr>
<tr><th>decimal point</tr>
</table>
  変数
<table>
<tr><th>variable<td>dt<td>wh<td>bh<td>wrps<td>werr<td>dbh</tr>
<tr><th>description<td>2&pi;<td>サンプリング時間<td>角速度<td>角速度(小数部切捨て)<td>&omega;推定誤差<td>極<td><i>b&times;2&pi;</i><td>b補正(後述)</tr>
<tr><th>bit width</tr>
<tr><th>decimal point</tr>
</table>
PRM_ReadDataは引数のアドレスにあるROMデータを参照する。戻り値は8ビット(0..255)である。  
このブロックは最初のサンプリングで1回だけ処理される。変数の初期化が行われる。  
b2pc はゼロ点補正部(ゼロ点のズレを修正)での使用のためROM値で初期化する
このブロックは毎サンプリング(1回転ごと)に処理される。先の離散化オブザーバの数値計算を行う。  
この計算途中の dbh は&beta;の微分として利用されるが、&Delta;tが乗じられていることを忘れないように。&beta;の微分とするには&Delta;tで除すること。

2. スライディングモード制御
---
  1. 技術解説
<img src="fig2.png" style="float:right;" >  
右図は、位相平面内の張力のローカスをイメージしたものである。スライディングモード制御は状態変数(張力と張力微分)の位相平面内の位置が、切替面(青線)の左右のどちらにあるかで、制御入力(PWMデューティ)を切り替える制御である。  
つまり、切替面から離れた位置では、系の閉ループゲインが０に対し、切替面近傍では∞のゲインを持つ。これにより不安定な系を安定化させる手法である。  
  1. コード解説
-->
