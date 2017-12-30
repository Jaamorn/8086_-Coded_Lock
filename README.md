# 一个基于 STAR ES598PCI 8086实验系统的密码锁
## 概述
* 这是一个可定时更换密码的密码锁
* 汇编代码基于星研集成环境
* 控制器代码基于恩智浦mk60dn512zvlq10单片机
* 使用一个74HC08芯片作为脉冲转换模块
* 电路原理图使用Altium Designer绘制
## 目录说明
| 文件/目录        | 说明           |
| :----------- | :----------- |
| Controler    | 密码控制器/显示器源代码 |
| Circuit      | 电路原理图        |
| 1616LED.xlsx | 显示屏取模说明      |
| lock.asm     | 汇编源代码        |

### 片选说明

|  芯片  | 片选号  |
| :--: | :--: |
| 8255 | CS1  |
| 8259 | CS2  |
| 8253 | CS3  |
| 8155 | CS4  |

### 接线说明

| 区域及引脚                 | 对应区域及引脚          |
| :-------------------- | :--------------- |
| E5区 CS,A0             | A3区 CS5,A0       |
| E5区 CLK               | B2区 2M           |
| E5区 A、B、C、D           | G5区 A、B、C、D      |
| B3区 CS A0             | A3区 CS2 A0       |
| B3区 INT,INTA          | ES8688 INTR INTA |
| B3区 IR0               | 同步信号             |
| B4区 CS,A0,A1          | A3区 CS1，A0，A1    |
| B4区 JP56              | G6区 JP65         |
| C5区 CS,A0,A1          | A3区 CS3,A0，A1    |
| C5区 CLK0              | B2区 2M           |
| C5区 OUT0              | C5区 CLK1         |
| C5区 OUT1,OUT2         | D1区 Ctrl         |
| C5区 GATE0，GATE1，GATE2 | C1区 VCC          |
| C5区CLK2               | B2区 1M           |
| B4区 CS,IO/M           | A3区 CS4,A8       |
| B4区 JP56,JP53         | A2区 JP23,JP24    |
| B4区 JP52，JP76         | A2区 JP33,JP34    |
## 特别致谢

* 特别感谢[这篇文章](http://jiajiewen.xyz/study/password-lock)的作者，为我们提供了思路，我们使用了这篇文章里的部分代码。
* 感谢队友张晟晖协助完成音频表的建立，声音提示部分代码的编写，以及系统的测试和Debug工作。