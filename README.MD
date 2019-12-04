# CSS3学习笔记

本项目下的每个文件都可以正常运行，参考笔记进行理解。

css选择器和jQuery的选择器大体一致，不知道是谁抄谁的，程序之间怎么叫抄呢，就像是孔乙己的“窃”，哈哈。
## 1.选择器

### 基本选择器

1. <选择器>,<选择器>  逗号 选择多个元素
3. <选择器>><选择器>  大于号 第一个选择器的直接后代（也就是仅选择儿子辈）且匹配第二个选择器
4. <选择器>+<选择器>  加号 紧挨第一个选择器且匹配第二个选择器的兄弟标签
5. <选择器>~<选择器>  波浪线 第一个选择器之后且匹配第二个选择器的所有兄弟们标签

### 属性选择器

1. 元素E[name]  选择具有name属性的元素
2. 元素E[name^=a]  选择属性name的值以a开头的元素，其中name为任意属性，a为任意值
3. 元素E[name$=d]  选择属性name的值以d结尾的元素，其中name为任意属性，d为任意值
4. 元素E[name*=o]  选择属性name的值包含o的元素，其中name为任意属性，o为任意值


### 伪类选择器
举几个个栗子。。。

1. p:empty 选择空的p元素
2. div p:not(.hello)  div中除了类为hello的p元素
3. ::selection  选择被用户选择的部分（鼠标拖动选择一部分）


### 子元素选择器
以p标签举例说明

1. p:nth-child(n)  选择p的父元素中第n个子元素,且这个子元素n为p
2. p:first-child   选择p的父元素中的第一个子元素，且这个子元素为p
3. p:last-child   选择p的父元素中的最后一个子元素，且这个子元素为p
4. p:nth-last-child(n)   选择p的父元素的倒数第n个子元素，且这个子元素为p
5. p:only-child   选择p元素的父元素中的唯一子元素，且这个元素为p
6. p:nth-of-type(n)  选择p的父元素中的第n个p元素
7. p:first-of-type   选择p的父元素中的第一个p元素
8. p:last-of-type    选择p的父元素中的最后一个p元素
9. p:only-of-type   选择p的父元素中的为一个p元素
10. p:nth-last-of-type(n)  选择p元素的父元素中倒数第n个p元素

**总结：**
> child和type类似，唯一的区别是child按次序要求的第n个元素必须是冒号前边的元素，而type仅仅是要求按次序的第n个元素。
>举个例子，早些时候，伟大的人类一般生育好多孩子，假设张三生了9个孩子，4个女儿，5个儿子，如果一个人要找“张三家的老大哥”，类比子元素选择器，如果使用nth-child(1)，那么找到的老大哥必须是张三的头胎老大，其余8个都是他的弟弟妹妹；如果用nth-of-type(1),那么找到的就是男孩子中的老大，这个老大可能是几个姐姐的弟弟。


### 文本选择器

以p举例子记录

1. p:first-line  选择p标签短路中的第一行
2. p:first-letter 选择p标签段落中的第一个字母
3. p:before  选择p的前面
4. p:after  选择p的后面

## 2.CSS3新增属性

### 颜色

1. rgba(红,绿,蓝，透明度)   设置颜色，其中红绿蓝为0-255之间的值，透明度为0-1 1为不透明，0.5为半透明

### 字体单位
字体单位分为三种，每种具体介绍如下：

1. px：使用具体像素点作为单位，好处是比较稳定和精确，但在浏览器中放大或者缩小页面会存在问题。
2. em：em是以父级为参考的单位，好处是字体可以自适应，但父元素标签发生改变时字体大小将不确定
3. rem：rem是相对于根元素《html》的文字大小，这样就意味着，我们只需要在根元素确定一个参考值即可,css3新增。当在不同屏幕下只需要更改《html》的文字大小，其他使用rem的元素会自动响应改变。



### 文本溢出隐藏

1. white-spance:nowrap  文本不换行
2. overflow: hidden;  溢出隐藏
2. text-overflow: ellipsis;  超出文本以省略号显示。
3. text-overflow: "###"  自定义显示，也可以是其他符号


### 文本阴影

1. text-shadow: h-shadow v-shadow blur color  水平阴影位置，垂直阴影位置，模糊距离，阴影颜色

阴影实际用的比较少，

### 盒子阴影

盒子阴影是一个很漂亮的效果，比如小米的产品块，之前自己一直想做的数据卡片，鼠标hover后展现阴影，显得有立体感，可以使用这个属性

1. box-shadow: h-shadow v-shadow blur spread color  水平位置 垂直位置 模糊距离 阴影尺寸 阴影颜色。

阴影其实是由一个实心块和一个渐变块组成的，实心块就是spread，阴影块就是blur，他们都不占空间位置。


### 盒子最大最小尺寸

当元素是以百分之多少设定的大小是，元素的大小会随着浏览器的缩放自动调整，那么就可以通过下列属性设定盒子的最大最小尺寸，当超过最大最小尺寸后，元素大小保持最大最小尺寸保持不变。

1. max-width  最高宽度
2. min width  最小宽度
3. max-height  最大高度
4. min-height  最小高度

### 盒子尺寸

一般默认设置一个块级元素的宽高（width、height）指的是元素内部内容的宽高，不包括内边距padding和边框border。而box-sizeing属性可以设置width包括pading和border。此时也称为**怪异模式**。

1. box-sizing: border-box | content-box


其中border-box设置width包含border和width，也就是怪异模式。
content-box就是正常的width只包含内容

**怪异模式**很有用，比如在页面中平均排列五个块级元素，如果每个块级元素由边框，即使设为每个宽度为20%也不行，因为有边框一行放不下，此时设置boxsizing为border-box，width的20%包含边框，无论边框多大，都能现在在一行内。

### 分栏效果
分栏和报纸中的效果一致，直接使用下面相关属性就可以

1. column-width  栏宽
2. column-count  列数
3. column-gap    列间距 参数和border一样
4. column-rule   分割线

另外有两个前缀
1. -moz-  是针对firefox的设置，
2. -webkit- 是针对chrome和苹果safari的设置
3. -o- Opera浏览器
4. -ms-  ie浏览器

### 背景图尺寸

背景图尺寸是CSS3新增属性

1. background-size  宽度 高度 参数的宽高可以是n%或者具体的像素值。

### 背景线性渐变

除了背景图和背景颜色还可以设置背景线性渐变,利用background的数值值liner-gradient实现 （其中gradient是梯度的意思）

1. background: linear-gradient(开始位置，渐变颜色[，渐变颜色，渐变颜色])  后加多个渐变颜色。

这个属性有兼容性问题，必须使用特定浏览器的前缀才能被浏览器识别。比如在谷歌浏览器下，其属性为  -webkit-liner-gradient

### 轮廓线

盒子轮廓线和盒子的边框border的效果一样，唯一不同的是盒子轮廓线outline不占用空间。

1. outline: solid 2px red; 参数和border一样。


### 动画
CSS3可以做动画了，不必像之前使用jquery中的 animaate了

1. transition ： width 2s;  

transition 这个单词的意思是过度，用于监控属性的变化，如果被监控的属性发生变化，那么就是通过运动，运动过去。其中“width”是要监控的属性，“2s”是变化时间为2秒，也可直接把要监控的属性写为all，这是懒人方法。

transition不知能实现CSS控制的动画，也能监控js做出属性改变，实现动画。

### 元素变形
元素变形使用transform属性，支持旋转

#### 旋转

1. transform： rotate(30deg);  旋转30度，deg为角度，默认顺时针方向

元素旋转后还是占用原来的位置，旋转默认的基准点是中心店，基准点可调，后边介绍。

指定旋转的原点：

1. transform-origin: 4px 25px;/*这里的像素值分别是对旋转元素左上角和距离：距离上边，距离左边*/


#### 缩放

1. transform: scale(1.5,1,5); //xy方向分别放大1.5倍


#### 扭曲

1. transform: skew(0,30deg);  //垂直扭曲30度，内容文字也会被扭曲


#### 平移
平移和相对定位差不多

1. transform: translate(100px,20px);  //往下100px，往右平移100px

平移和定位的区别是平移没有脱离文档流,元素就算是定位的，也可以平移。


### 元素水平垂直居中
要使元素水平垂直居中有三种方式，分别如下：
#### 第一种：
前提是元素有固定的宽高， 这种方式使用绝对定位，然后设置top，bottom，left，right都为0，再设置margin为auto就能实现自动水平垂直居中

    position: absolute;
    top:0px;
    left:0px;
    right:0px;
    bottom:0px;
    margin:auto;
 
#### 第二种：
前提是元素有固定的宽高，
这种方式先定位，左右偏移50%，然后再把margin-top和margin-left设为元素宽高的负一半的值

    position: absolute;
    top:50%;
    left:50%;
    margin-top:-元素高度的一半px;
    margin-left:-元素宽度的一半px;

#### 第三种：
这种方式适用与元素有宽度值一定，高度值是靠内部元素撑起来的，也就是高度不确定的情况，这里用到了元素平移。

    position: absolute;
    top:50%;
    left:50%;
    /*注意：下方的50%是只box元素本身宽高的50%，而上方的50%是指box父元素宽高的50%*/
    transform: translate(-50%,-50%);



### animation关键帧运动
animation关键帧运动效果和transition实现效果差不多，不过这种运动就是在不经过触发自己就能执行的运动，分为两步，第一：指定关键帧开始和结束的状态，第二：在选择器中指定运动规则。


```html
<!-- 设置运动规则： -->
@keyframes name{ /*指定运动规则的名称为name*/
    from{
        css属性
    }
    to{
        css属性
    }
}

<!-- 运动规则还可以指定关键帧的状态，如下
 -->
 @keyframes name{
     0%{
          css属性
     }
     50%{
          css属性
     }
     100%{
          css属性
     }
 }

<!-- 指定运动规则： -->
选择器{
    animation-name：name;//规则名称
    animation-duration：3s;//运动时间
    <!--最少有以上两个： 规则名称和持续时间就能运动 -->

    animation-timing-function:linear;//匀速变化，默认是变速
    animation-delay：2s;//延迟两秒执行
    animation-iteration-count：3;//运动次数，指定infinite就是无限重复运动。
    animation-directioin:alternate;//轮流反相运动（见孙悟空旋转金箍棒）
    animation-fill-mode: forwards | backwards | both; //运动填充模式 比较少用
    <!-- 
        forwards：运动结束后保持运动结束的状态 
        backwards:在延迟期间保持开始运动的状态
        both：表示forwards和backwards同时存在 -->

    <!-- 运动的暂停 -->
    animation-play-status:pause; //运动暂停

    <!-- 还可将上述的animation写到一起 -->
    animation： name time linaer 1s infinite;//名称，时间，运动形式，延迟时间，运动次数
}

```

### flex弹性盒子布局

#### 基本概念

* flex是flexible box的缩写，意为“弹性布局
，用来为盒装模型提供最大的灵活性。
* 任何一个容器都可以指定为flex布局。
* 采用flex布局的元素，成为Flex容器（flex container）简称"**容器**“，他的所有子元素都自动成为容器成员，成为flex项目（flex item），简称”**项目**“

* 容器默认存在两根轴，水平叫”主轴“（main axis）和垂直的交叉轴（cross axis）起始位置和结束位置分别成为：main start，main end,cross start cross end;

* 项目默认沿着主轴排列，单个项目占用的主轴控件叫做main size，占据的交叉轴控件叫做cross size。

#### 特点
1. 弹性盒模型容器中的项目默认在主轴上排列
2. 如果项目总宽度，小于容器宽度，按照设置的宽度正常排列;如果项目总宽度大于容器宽度，那么项目宽度均分总宽度，设置的宽度就无效了。

#### 主轴方向
主轴方向可以定义，主轴和交叉轴永远是交叉的。

1. flex-direction: row-reverse|column|column-reverse 定义主轴方向

#### 元素在主轴上对齐方式
项目在容器的主轴上的对齐方式可以利用justify-content设置
1. justify-content: flex-start|flex-end|center|space-between|space-around；定义主轴上对齐方式，

分别是:
flex-start:从主轴开始起对齐排列，
flex-end:结束起对齐排列，
center:居中对齐排列，
space-between:两端对齐-项目之间间隔都相等
space-around:每个项目两侧间隔相等。

#### 元素在交叉轴上的对齐方式

1. align-items：flex-start|flex-end|center|baseline|stretch；

其中：
flex-start： 交叉轴的起点对齐。
flex-end：交叉轴的终点对齐
center：交叉轴的中点对齐
baseline：项目的第一行文字的基线对齐
stretch：（默认值）如果项目未设置高度或设置为auto，将占满整个容器的高度 （stretch的意思是拉伸）

#### 项目的换行

1.flex-wrap：wrap|wrap-reverse; 用于设置项目换行。

wrap：如果子元素也就是项目的总宽度大于父级元素也就是容器的宽度，那么一行能放多少则放多少，多余的放在下一行
wrap-reverse; 从下方开始换行
 
#### 多根轴线的对齐方式

多根轴线的排列就是多行项目的排列方式
 /* 这个属性没看懂 */

1. align-content: flex-start|flex-end|center|space-between|space-around；

#### 项目的顺序

1. order：3; 默认为0，越小排列越靠前

#### 项目的宽度比例
是项目之间的比例

1. flex：2 ； 默认为1，宽度的比例





