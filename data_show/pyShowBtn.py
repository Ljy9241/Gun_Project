# -*- coding: utf-8 -*-
"""
Created on Wed Mar  7 22:35:35 2018

@author: think
"""
import time
import serial
import serial.tools.list_ports
import pyqtgraph as pg
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QEvent, QObject
import array
import serial
import threading
import numpy as np
from queue import Queue

gQLen = 50000               #每1ms可能有10个字节，则可缓存0.5s
gCurInd = 0                 #每收到1帧数据，其值增1，回环；曲线的当时光标
qDat = Queue(maxsize=gQLen) #串口缓冲队列
qCmd = Queue(maxsize=10)    #PC的命令队列
recvInd=0                   #用于指示当前帧的第n个有效数据
historyLength = 1000        # 横坐标长度
sampleRate = 1
sampleInd = 0
lineAng = 1

#是一个独立的进程、线程，所以不能直接进行数的操作
def Serial():
    global gCurInd, recvInd, chksum,  headFlag, ifRecvDat
    #global gCurInd, qDat;
    while(True):
        time.sleep(0.01)    #这个一定要有，否则会占用很多CPU资源
        try:
            if mSerial.isOpen():
                n = mSerial.inWaiting()     # 得到待读取的数据个数
                #if(n):     
                for i in range(n):
                    dat = int.from_bytes(mSerial.readline(1), byteorder='little')  # 格式转换   这里是读1行？应该是读1个字节才对吧 超时1ms？
                    #dat = int.from_bytes(mSerial.read(), byteorder='little')  # 格式转换   这里是读1行？应该是读1个字节才对吧
                    if ifRecvDat==1:            #只有在全能接收后，才放到缓存里，否则不放
                        qDat.put_nowait(dat)   #不阻塞，如果已满，则 exception   qDat.put(dat) 则是阻塞的，即满了的话会一直等待
            else:
                time.sleep(0.1)
        except:
            time.sleep(0.1)
#是一个独立的进程、线程，所以不能直接进行数的操作
def Keyscan():
    global keypress
    while(True):                        #每隔0.01秒扫描一下按键处理
        time.sleep(0.01)
        cmd = input('input cmd:')
        qCmd.put(cmd)       #可以把指令一次性压入

#from datetime import datetime
import datetime
import copy

chksum = 0
headFlag = 0
ifRecvDat = 1
ifChkDic={'sp':1,'pv':1,'mv':1,'err':0,'amv':0,'auto':1,'sendHex':0,'recvHex':0}
psParaDic={'posNeg':'正作用', 'lineAng':'直行程', 'sigMode':'4-20mA'}
def plotData():
    global gCurInd, recvInd, chksum, headFlag, ifRecvDat
    global datInd0, datInd1, datInd2, datInd4, datInd5
    global sampleInd, sampleRate

    #print('.')
    #逐一取出数据，进行校验判断后，存储到相应缓存内
    while(not qDat.empty()):
        dat = qDat.get()
        xx = txtRecv.text()[-100:]
        if ifChkDic['recvHex']:     # hex显示
            txtRecv.setText(xx+' '+str(hex(dat))[2:])
        else:
            txtRecv.setText(xx+chr(dat))
        #print(dat)
        if ifRecvDat==0:
            continue
        #if dat&0b10000000 and dat&0x0F:     # 最高位为1， 且有触发
        if dat==0xff:                   # 最高位为1
            sampleInd += 1
            if sampleInd>=sampleRate:   #根据采样率来决定要不要加入后续处理。未到采样点时，相当于未收到头，数据丢失
                sampleInd = 0                
                recvInd = 0
                headFlag =1
            #print('head', dat)
        elif headFlag:                       #只有收到过头后，才接收
            if (recvInd == 0):                      #数据未收满，暂存
                    dataSp[gCurInd] = dat
            elif recvInd ==1:
                    dataPv[gCurInd] = dat
                    dataErr[gCurInd] = dataSp[gCurInd]-dataPv[gCurInd]
            elif recvInd==2:
                    dataMv[gCurInd] = dat
                    gCurInd = (gCurInd +1) % historyLength
            else:
                headFlag = 0   
            recvInd += 1
  
    #这里是对命令行进行处理
    while(not qCmd.empty()):
        cmd = qCmd.get()
        cmdHead = cmd[0]         #取得指令头，下面进行分别处理
        if cmdHead == 'w':       #保存数据，将数据写入到指定的文件中
            name = cmd[1:]
            nowTime=datetime.datetime.now().strftime('%Y%m%d%H%M%S')#现在
            filepath = 'S:/'+nowTime+'-'+name+'.txt'    #默认还会加上时间戳
            print('save file:', filepath)
            with open(filepath, 'w+') as f:         #这是旧方法，逐一变换写入
                f.write('A:' + str(list(dataSp))[1:-1])
                f.write('\nB:' + str(list(dataPv))[1:-1])
                f.write('\nC:'+ str(list(dataMv))[1:-1])
        elif cmdHead == 'r':       #加载数据， 加载指定文件的数据
            name = cmd[1:]
            filepath = 's:/'+name+'.txt'
            print('read file:', filepath)
            with open(filepath, 'r') as f:
                txt = f.readlines()
                aa=eval(txt[0][2:])     #由于是有 A： B： C：开头，先去除
                bb=eval(txt[1][2:])
                cc=eval(txt[2][2:])
                if len(aa)>=historyLength and len(aa)==len(bb)==len(cc):    #防止错误格式，或指定了不合适的文件
                    for i in range(historyLength):
                        dataSp[i]=aa[i]
                        dataPv[i]=bb[i]
                        dataMv[i]=cc[i]
                else:
                    print('指定的文件格式不规范')                
        elif cmd == 'start' or cmd=='t':        #启动记录，数据清0,坐标回起点
            ifRecvDat = 1
            for i in range(historyLength):
                dataSp[i]=1
                dataPv[i]=63
                dataMv[i]=127
        elif cmd == 'stop' or cmd=='p':        #停止记录
            ifRecvDat = 0
        else:
            print('err cmd')

    #到这里数据已经全部放好                
    
    _=curveSp.setData(dataSp) if ifChkDic['sp'] else curveSp.clear()
    _=curvePv.setData(dataPv) if ifChkDic['pv'] else curvePv.clear()
    _=curveMv.setData(dataMv) if ifChkDic['mv'] else curveMv.clear()
    _=curveErr.setData(dataErr) if ifChkDic['err'] else curveErr.clear()
    _=curveAMv.setData(dataAMv) if ifChkDic['amv'] else curveAMv.clear()
    #_=curveSpmid.setData(dataSpmid) if ifChkDic['sp'] else 0
    #_=curvePvmid.setData(dataPvmid) if ifChkDic['pv'] else 0
    #_=curveMvmid.setData(dataMvmid) if ifChkDic['mv'] else 0


gDiv = 1
xTestModZip, yTestModZip, zTestModZip = 0, 0, 0
oriZipLen = 10000
modZipLen = 0
mSerial=0
# 打开、关闭 串口
def comboBoxCommChanged():
    global mSerial
    nowCom = comboBoxComm.currentText()
    try:
        if nowCom!=mSerial.port:
            print('switch')
            #mSerial.close()
            btnStop.setText('OPEN')
        else:
            print('no change')    
        print(nowCom)
    except:
        print('no mSerial.')
# 改变采样率
def comboBoxRateChanged():
    global sampleRate
    sampleRate = int(comboBoxRate.currentText())
    print(sampleRate)

# 改变直行程、角行程 
def comboBoxLineAngChanged():
    if psParaDic['lineAng'] !=comboBoxLineAng.currentText():
        psParaDic['lineAng'] = comboBoxLineAng.currentText()
        print(psParaDic['lineAng'])
        if psParaDic['lineAng']=='直行程':
            cmd = [0x06]+[ord(x) for x in '0000']+[0x0A]    
        else:   #if psParaDic['lineAng']=='角行程':
            cmd = [0x06]+[ord(x) for x in '0001']+[0x0A]    
        xx = bytes(cmd)
        mSerial.write(xx)

#改变 正、反作用
def comboBoxPosNegChanged():
    if psParaDic['posNeg'] !=comboBoxPosNeg.currentText():
        psParaDic['posNeg'] = comboBoxPosNeg.currentText()
        print(psParaDic['posNeg'])
        if psParaDic['posNeg']=='正作用':
            cmd = [0x07]+[ord(x) for x in '0000']+[0x0A]    
        else:   #if psParaDic['lineAng']=='反作用':
            cmd = [0x07]+[ord(x) for x in '0001']+[0x0A]    
        xx = bytes(cmd)
        mSerial.write(xx)

#改变 信号制式
def comboBoxSigModeChanged():
    if psParaDic['sigMode'] !=comboBoxSigMode.currentText():
        psParaDic['sigMode'] = comboBoxSigMode.currentText()
        print(psParaDic['sigMode'])
        if psParaDic['sigMode']=='4-20mA':
            cmd = [0x08]+[ord(x) for x in '0000']+[0x0A]    
        elif psParaDic['sigMode']=='4-12mA':
            cmd = [0x08]+[ord(x) for x in '0001']+[0x0A]    
        else:
            cmd = [0x08]+[ord(x) for x in '0002']+[0x0A]    
        xx = bytes(cmd)
        mSerial.write(xx)


hexChr='0123456789abcdefgABCDEFG'
dicHexCharToDec={'0':0,'1':1,'2':2,'3':3,'4':4,'5':5,'6':6,'7':7,'8':8,'9':9,'a':10,'b':11,\
    'c':12,'d':13,'e':14,'f':15,'A':10,'B':11,'C':12,'D':13,'E':14,'F':15,}

# 按钮统一入口
def btnPress(sel):
    global mSerial, gCurInd
    print('btn:'+sel)
    try:
        # 从头开始显示
        if 'RST'==sel: 
            gCurInd = 0
        #电流校准    
        if 'cabI'==sel:      
            if mSerial.isOpen():
                #cmd = 'cabI:'+txtCabI.text()+'\n'       # # QTextEdit 用 toPlainText
                txt = str(int(float(txtCabI.text())*10)).zfill(4)
                cmd = [0x04]+[ord(x) for x in txt]+[0x0A]
                xx = bytes(cmd)
                #print(xx)
                mSerial.write(xx)  
        #阀位校准
        if 'cabPv'==sel:     
            if mSerial.isOpen():
                #cmd = 'cabPv:'+txtCabPv.text()+'\n'      #
                txt = str(int(float(txtCabPv.text())*10)).zfill(4)
                cmd = [0x05]+[ord(x) for x in txt]+[0x0A]
                xx = bytes(cmd)
                mSerial.write(xx)  
        #设定SP
        if 'setSp'==sel:                         # MCU将不再检测电流，直接采用PC设定的SP；但仍会进行PID控制，可视为半自动
            if mSerial.isOpen():
                #cmd = 'sp:'+txtSp.text()+'\n'
                #xx = bytes(cmd, 'utf-8')
                txt = str(int(float(txtSp.text())*10)).zfill(4)
                xx = bytes([0x02]+[ord(x) for x in txt]+[0x0A])
                mSerial.write(xx) 
        #设定MV
        if 'setMv'==sel:                         # MCU将不再进行PID控制，直接采用PC设定的MV。 取消 AUTO 选项
            if mSerial.isOpen():
                #cmd = 'mv:'+txtMv.text()+'\n'   
                #xx = bytes(cmd, 'utf-8')
                txt = str(int(float(txtMv.text())*10)).zfill(4)
                xx = bytes([0x03]+[ord(x) for x in txt]+[0x0A])
                mSerial.write(xx)
                chkShowAuto.setChecked(False)
        #向MCU发送指令
        if 'sendMcu'==sel:              
            if mSerial.isOpen():
                if ifChkDic['sendHex']:
                    oriList=txtSend.text().split()      #注意十六进制必须以空格间隔
                    xx=bytes([])
                    for x in oriList:
                        if len(x)==1 and x in hexChr:                           #单个字符，前面补0处理
                            xx += bytes([dicHexCharToDec[x]])
                        elif len(x)==2 and x[0] in hexChr and x[1] in hexChr:   #两个字符
                            xx += bytes([dicHexCharToDec[x[0]]*16 + dicHexCharToDec[x[1]]])
                    mSerial.write(xx)
                else:                                   #非十六进制模式时，尾部加 '\n' 用以指示帧结束
                    cmd = txtSend.text()+'\n'
                    xx = bytes(cmd, 'utf-8')            #转为字节流
                    mSerial.write(xx)
        #向PC发送指令
        if 'sendPc'==sel:               
            cmd = txtSend.text()
            if len(cmd)>0:
                qCmd.put(cmd)       #可以把指令一次性压入
    except:
        QMessageBox.information(win,"错误","输入数据错误 或 串口尚未打开！",QMessageBox.Yes)
# 打开、关闭串口处理
def btnStopPress():
    global mSerial  
    if btnStop.text()=='OPEN':      #原来处于关闭状态
        nowCom = comboBoxComm.currentText()
        print(nowCom)
        try:
            mSerial = serial.Serial(nowCom, int(115200), timeout=10, parity=serial.PARITY_NONE, stopbits=1)  #等待超时时间单位为s,，即最多等待此时间
            if (mSerial.isOpen()):
                btnStop.setText('CLOSE')
                print('open',nowCom, 'OK')
            else:
                serial.close()
                print('open',nowCom, 'Fail')
        except:
            print('open',nowCom, 'Fail')
            QMessageBox.information(win,"错误","串口无法打开，请确认串口号！",QMessageBox.Yes)

    else:                           #CLOSE 原来处于打开状态
        mSerial.close()
        btnStop.setText('OPEN')
    
# 复选框 处理入口
def chkChgPress(sel):
    print('chk:'+sel)
    ifChkDic[sel] = (ifChkDic[sel]+1)%2
    if sel=='auto':     #自动选项
        if ifChkDic['auto']==1:     # 选中，发送切换指令。 本程序仅显示
            if mSerial.isOpen():
                xx = bytes([0x01]+[ord(x) for x in '0000']+[0x0A])
                mSerial.write(xx)
        else:                       # 取消，发送切换指令。 根据本程序的设定SP、MV等工作；注意 设定SP、MV时，将自动取消勾选
            if mSerial.isOpen():
                xx = bytes([0x01]+[ord(x) for x in '0001']+[0x0A])
                mSerial.write(xx)

#参考自： https://blog.csdn.net/lanceleng/article/details/108914819 
#想实现上、下键的识别响应
class CustomQLineEditWrapper(QObject):        
    def __init__(self, editor):
        # installEventFilter 依赖QObject，因此必须先调用父类构造函数
        super().__init__()
        self.editor = editor
        self.editor.installEventFilter(self)
        self.keyMap = {}
 
    def setKeyPressCallback(self, key, callback):
        self.keyMap[key] = callback
 
    # 返回True表示将此消息消耗掉，返回False则不会
    def eventFilter(self, obj, ev):
        if obj is not self.editor:
            return False
        if ev.type() != QEvent.KeyPress:
            return False
        if ev.key() in self.keyMap:
            self.keyMap[ev.key()]()
            return True
        return False


def eventFilter(event):
    print(event)

gCommCnt=0
#if 0:
if __name__ == "__main__":
    app = pg.mkQApp()                               # 建立app
    win = pg.GraphicsWindow()                       # 建立窗口
    win.setWindowTitle(u'PS Assistant V1.0')
    win.resize(1100, 768)                            # 小窗口大小
    #grid = pg.QtGui.QGridLayout()
    #win.setLayout(pg.QtGui.QGridLayout())
    #p3=win.layout()
    #widgets.setLayout(win)
    #win.rotate(90)
    #historyLength = 64                             # 横坐标长度
    #pxyz = win.addPlot(row=1, col=1, rowspan=2, colspan=2)                               # 把图p加入到窗口中
    pxyz = win.addPlot(row=0,col=0)                   # 把图p加入到窗口中，第0行第0列
    #pxyz = p3.addPlot(row=0,col=0,rowspan=6,colspan=6)                   # 把图p加入到窗口中，第0行第0列
    #pxyz = win.addPlot(row=0,col=0,colspan=5)                   # 把图p加入到窗口中，第0行第0列
    pxyz.showGrid(x=True, y=True)                      # 把X和Y的表格打开
    #pxyz.setRange(xRange=[0, 64], yRange=[0, 128], padding=0)
    #pxyz.setLabel(axis='left', text='y / V')          # 靠左,靠下 标识栏
    #pxyz.setLabel(axis='bottom', text='x / point')
    #pxyz.setTitle('semg')                              # 表格的名字

    p3 = win.addLayout(row=2,col=0)     
    p1 = win.addLayout(row=1, col=0)    #顺序在后，解决下拉菜单遮挡问题
    #wg = pg.QtGui.QGraphicsProxyWidget()
    #p3.setLayout(pg.QtGui.QGridLayout())

    #p3 = win.addLable()
    #p3.setLayout(pg.QtGui.QGridLayout())
    
    #串口号 下拉框
    proxyComboBoxComm = pg.QtGui.QGraphicsProxyWidget()
    comboBoxComm = pg.QtGui.QComboBox()
    for i in range(4):
        comboBoxComm.addItem('')
        comboBoxComm.setItemText(i,'COM'+str(i+1))
    comboBoxComm.currentIndexChanged.connect(comboBoxCommChanged)
    proxyComboBoxComm.setWidget(comboBoxComm)
    #打开、关闭串口
    proxyBtnStop = pg.QtGui.QGraphicsProxyWidget()
    btnStop = pg.QtGui.QPushButton("OPEN", clicked=lambda:btnStopPress())
    btnStop.setFixedWidth(40)
    btnStop.setFixedHeight(21)
    proxyBtnStop.setWidget(btnStop)
    #自动运行，勾选时，表示完全单片机运行，包括电流检测及控制
    proxyChkBoxAuto = pg.QtGui.QGraphicsProxyWidget()
    chkShowAuto = pg.QtGui.QCheckBox('AUTO', clicked=lambda:chkChgPress('auto'))
    chkShowAuto.setChecked(True)
    chkShowAuto.setFixedHeight(20)
    proxyChkBoxAuto.setWidget(chkShowAuto)
    #曲线从头开始显示， 便于截图等分析记录
    proxyBtnRst = pg.QtGui.QGraphicsProxyWidget()
    btnRst = pg.QtGui.QPushButton("RST", clicked=lambda:btnPress('RST'))
    btnRst.setFixedWidth(30)
    btnRst.setFixedHeight(21)
    proxyBtnRst.setWidget(btnRst)
    #手动设置 SP 的值，可以不用去调整信号源，方便调试
    proxyTxtSp = pg.QtGui.QGraphicsProxyWidget()
    txtSp = pg.QtGui.QLineEdit("50")
    txtSp.setFixedWidth(35)
    #txtSp = CustomQLineEditWrapper(txtSp)
    proxyTxtSp.setWidget(txtSp)


    #手动设置 SP 按钮，按下后发送 txtSp 中的文本作为SP
    proxyBtnSetSp = pg.QtGui.QGraphicsProxyWidget()
    btnSetSp = pg.QtGui.QPushButton("setSP", clicked=lambda:btnPress('setSp'))
    btnSetSp.setFixedWidth(40)
    btnSetSp.setFixedHeight(21)
    proxyBtnSetSp.setWidget(btnSetSp)
    #手动设置 MV 的值，可以开环情况下测试全开、全关或中间位置
    proxyTxtMv = pg.QtGui.QGraphicsProxyWidget()
    txtMv = pg.QtGui.QLineEdit("50")
    txtMv.setFixedWidth(35)
    proxyTxtMv.setWidget(txtMv)
    #手动设置 MV 的按钮，按下后发送 txtMv 中的文本作为MV
    proxyBtnSetMv = pg.QtGui.QGraphicsProxyWidget()
    btnSetMv = pg.QtGui.QPushButton("setMV", clicked=lambda:btnPress('setMv'))
    btnSetMv.setFixedWidth(40)
    btnSetMv.setFixedHeight(21)
    proxyBtnSetMv.setWidget(btnSetMv)
    # 曲线显示 选项 SP
    proxyChkBoxSp = pg.QtGui.QGraphicsProxyWidget()
    chkShowSp = pg.QtGui.QCheckBox('SP', clicked=lambda x:chkChgPress('sp'));       chkShowSp.setFixedHeight(20)
    chkShowSp.setChecked(True)
    proxyChkBoxSp.setWidget(chkShowSp)
    # 曲线显示 选项 PV
    proxyChkBoxPv = pg.QtGui.QGraphicsProxyWidget()
    chkShowPv = pg.QtGui.QCheckBox('PV', clicked=lambda x:chkChgPress('pv'));       chkShowPv.setFixedHeight(20)
    chkShowPv.setChecked(True)
    proxyChkBoxPv.setWidget(chkShowPv)
    # 曲线显示 选项 MV
    proxyChkBoxMv = pg.QtGui.QGraphicsProxyWidget()
    chkShowMv = pg.QtGui.QCheckBox('MV', clicked=lambda x:chkChgPress('mv'));       chkShowMv.setFixedHeight(20)
    chkShowMv.setChecked(True)
    proxyChkBoxMv.setWidget(chkShowMv)
    # 曲线显示 选项 ERR
    proxyChkBoxErr = pg.QtGui.QGraphicsProxyWidget()
    chkShowErr = pg.QtGui.QCheckBox('ERR', clicked=lambda x:chkChgPress('err'));       chkShowErr.setFixedHeight(20)
    proxyChkBoxErr.setWidget(chkShowErr)
    # 曲线显示 选项 AMV
    proxyChkBoxAMv = pg.QtGui.QGraphicsProxyWidget()
    chkShowAMv = pg.QtGui.QCheckBox('AMV', clicked=lambda x:chkChgPress('amv'));       chkShowAMv.setFixedHeight(20)
    proxyChkBoxAMv.setWidget(chkShowAMv)
    #电流校准 输入框
    proxyTxtCabI = pg.QtGui.QGraphicsProxyWidget()
    txtCabI = pg.QtGui.QLineEdit("75")
    txtCabI.setFixedWidth(35)
    proxyTxtCabI.setWidget(txtCabI)
    #电流校准 按钮
    proxyBtnCabI = pg.QtGui.QGraphicsProxyWidget()
    btnCabI = pg.QtGui.QPushButton("CabI", clicked=lambda:btnPress('cabI'))
    btnCabI.setFixedWidth(40);       btnCabI.setFixedHeight(21)
    proxyBtnCabI.setWidget(btnCabI)
    #阀位校准 输入框
    proxyTxtCabPv = pg.QtGui.QGraphicsProxyWidget()
    txtCabPv = pg.QtGui.QLineEdit("75")
    txtCabPv.setFixedWidth(35)
    proxyTxtCabPv.setWidget(txtCabPv)
    #阀位校准 按钮
    proxyBtnCabPv = pg.QtGui.QGraphicsProxyWidget()
    btnCabPv = pg.QtGui.QPushButton("CabPv", clicked=lambda:btnPress('cabPv'))
    btnCabPv.setFixedWidth(40);       btnCabPv.setFixedHeight(21)
    proxyBtnCabPv.setWidget(btnCabPv)
    #采样率提示标签
    proxyLabelRate = pg.QtGui.QGraphicsProxyWidget()
    labelRate = pg.QtGui.QLabel(" Rate")
    labelRate.setFixedWidth(35);            labelRate.setFixedHeight(21)
    proxyLabelRate.setWidget(labelRate)
    #采样率 下拉框
    proxyComboBoxRate = pg.QtGui.QGraphicsProxyWidget()
    comboBoxRate = pg.QtGui.QComboBox()
    for i in range(6):
        comboBoxRate.addItem('');        comboBoxRate.setItemText(i,str(2**i))
    comboBoxRate.setFixedWidth(35)
    comboBoxRate.currentIndexChanged.connect(comboBoxRateChanged)
    proxyComboBoxRate.setWidget(comboBoxRate)
    #直行程、角行程 下拉框
    proxyComboBoxLineAng = pg.QtGui.QGraphicsProxyWidget()
    comboBoxLineAng = pg.QtGui.QComboBox()
    comboBoxLineAng.addItem('');        comboBoxLineAng.setItemText(0,'直行程')
    comboBoxLineAng.addItem('');        comboBoxLineAng.setItemText(1,'角行程')
    comboBoxLineAng.setFixedWidth(60)
    comboBoxLineAng.currentIndexChanged.connect(comboBoxLineAngChanged)
    proxyComboBoxLineAng.setWidget(comboBoxLineAng)
    #正作用、反作用 下拉框
    proxyComboBoxPosNeg = pg.QtGui.QGraphicsProxyWidget()
    comboBoxPosNeg = pg.QtGui.QComboBox()
    comboBoxPosNeg.addItem('');        comboBoxPosNeg.setItemText(0,'正作用')
    comboBoxPosNeg.addItem('');        comboBoxPosNeg.setItemText(1,'反作用')
    comboBoxPosNeg.setFixedWidth(60)
    comboBoxPosNeg.currentIndexChanged.connect(comboBoxPosNegChanged)
    proxyComboBoxPosNeg.setWidget(comboBoxPosNeg)
    #信号制式 4-20 4-12 12-20 下拉框
    proxyComboBoxSigMode = pg.QtGui.QGraphicsProxyWidget()
    comboBoxSigMode = pg.QtGui.QComboBox()
    comboBoxSigMode.addItem('');        comboBoxSigMode.setItemText(0,'4-20mA')
    comboBoxSigMode.addItem('');        comboBoxSigMode.setItemText(1,'4-12mA')
    comboBoxSigMode.addItem('');        comboBoxSigMode.setItemText(2,'12-20mA')
    comboBoxSigMode.setFixedWidth(60)
    comboBoxSigMode.currentIndexChanged.connect(comboBoxSigModeChanged)
    proxyComboBoxSigMode.setWidget(comboBoxSigMode)

    proxyComboBoxSigMode
    #-------------------------------------------------------以下 P3 内容
    #接收显示， HEX 选项
    proxyChkBoxRecvHex = pg.QtGui.QGraphicsProxyWidget()
    chkRecvHex = pg.QtGui.QCheckBox('HEX', clicked=lambda x:chkChgPress('recvHex')); 
    chkRecvHex.setFixedWidth(40)
    chkRecvHex.setFixedHeight(20)
    proxyChkBoxRecvHex.setWidget(chkRecvHex)
    #接收区清空按钮
    proxyBtnClrRecv = pg.QtGui.QGraphicsProxyWidget()
    btnClrRecv = pg.QtGui.QPushButton("clrRecv ", clicked=lambda:txtRecv.setText(''))
    btnClrRecv.setFixedWidth(50)
    btnClrRecv.setFixedHeight(21)
    proxyBtnClrRecv.setWidget(btnClrRecv)
    #接收区 文本框
    proxyTxtRecv = pg.QtGui.QGraphicsProxyWidget()
    txtRecv = pg.QtGui.QLineEdit("textRecv")
    proxyTxtRecv.setWidget(txtRecv)
    #发送MCU区， HEX 选项
    proxyChkBoxSendHex = pg.QtGui.QGraphicsProxyWidget()
    chkSendHex = pg.QtGui.QCheckBox('HEX', clicked=lambda x:chkChgPress('sendHex')); 
    chkSendHex.setFixedWidth(40);           chkSendHex.setFixedHeight(20)
    proxyChkBoxSendHex.setWidget(chkSendHex)
    #发送MCU区， 按钮
    proxyBtnSendMcu = pg.QtGui.QGraphicsProxyWidget()
    btnSendMcu = pg.QtGui.QPushButton("->MCU ", clicked=lambda:btnPress('sendMcu'))
    btnSendMcu.setFixedWidth(50);           btnSendMcu.setFixedHeight(20)
    proxyBtnSendMcu.setWidget(btnSendMcu)
    #发送MCU区 文本框
    proxyTxtSend = pg.QtGui.QGraphicsProxyWidget()
    txtSend = pg.QtGui.QLineEdit("textSend", placeholderText="Press some words...")
    txtSend.returnPressed.connect(lambda:btnPress('sendMcu'))
    #txtSend.installEventFilter(win, eventFilter)               #原想绑定上、下键，未成功
    #txtSend.setKeyPressCallback(lambda:btnPress('sendMcuUp'))  #不支持的方法
    proxyTxtSend.setWidget(txtSend)
    #发送PC区， 按钮
    proxyBtnSendPc = pg.QtGui.QGraphicsProxyWidget()
    btnSendPc = pg.QtGui.QPushButton("->PC", clicked=lambda:btnPress('sendPc'))
    btnSendPc.setFixedWidth(40)
    btnSendPc.setFixedHeight(20)
    proxyBtnSendPc.setWidget(btnSendPc)
    #发送PC区 文本框
    proxyTxtCmd = pg.QtGui.QGraphicsProxyWidget()
    txtCmd = pg.QtGui.QLineEdit("textCmd", placeholderText="Press some words...")
    txtCmd.returnPressed.connect(lambda:btnPress('sendPc'))
    proxyTxtCmd.setWidget(txtCmd)
    #Other 按钮
    proxyBtnOther = pg.QtGui.QGraphicsProxyWidget()
    btnClrOther = pg.QtGui.QPushButton("Other", clicked=lambda:txtSend.setText(''))
    btnClrOther.setFixedWidth(40)
    btnClrOther.setFixedHeight(21)
    proxyBtnOther.setWidget(btnClrOther)
    #状态显示区 文本框
    proxySta = pg.QtGui.QGraphicsProxyWidget()
    sta = pg.QtGui.QLineEdit("sta")
    proxySta.setWidget(sta)

    '''
    #以下原计划设计单选组，未成功
    btng = pg.QtGui.QButtonGroup()
    selPc = pg.QtGui.QRadioButton('AUTO')
    selMan = pg.QtGui.QRadioButton('MAN')
    btng.addButton(selPc)
    btng.addButton(selMan)
    #p1.addItem(btng)
    proxyBtng.setWidget(selPc)
    '''

    p1.addItem(proxyComboBoxComm, row=0, col=0)    
    p1.addItem(proxyBtnStop, row=0, col=1)
    p1.addItem(proxyChkBoxAuto,row=0, col=2)
    p1.addItem(proxyBtnRst,row=0, col=3)
    p1.addItem(proxyTxtSp, row=0, col=4)
    p1.addItem(proxyBtnSetSp, row=0, col=5)
    p1.addItem(proxyTxtMv, row=0, col=6)
    p1.addItem(proxyBtnSetMv, row=0, col=7)
    p1.addItem(proxyChkBoxSp, row=0, col=8) 
    p1.addItem(proxyChkBoxPv, row=0, col=9) 
    p1.addItem(proxyChkBoxMv, row=0, col=10) 
    p1.addItem(proxyChkBoxErr, row=0, col=11)
    p1.addItem(proxyChkBoxAMv, row=0, col=12)
    p1.addItem(proxyTxtCabI, row=0, col=13)
    p1.addItem(proxyBtnCabI, row=0, col=14)
    p1.addItem(proxyTxtCabPv, row=0, col=15)
    p1.addItem(proxyBtnCabPv, row=0, col=16)
    p1.addItem(proxyLabelRate, row=0, col=17)    
    p1.addItem(proxyComboBoxRate, row=0, col=18)    
    p1.addItem(proxyComboBoxLineAng, row=0, col=19)    
    p1.addItem(proxyComboBoxPosNeg, row=0, col=20)    
    p1.addItem(proxyComboBoxSigMode, row=0, col=21)    

    '''
    #原计划设备统一列宽，并可自适应调整，未成功
    for i in range(p1.columnCount()):
        p1.setColumnStretch(i,1)
        p1.setColumnMinimumWidth(i,50)
    '''
    p3.addItem(proxyChkBoxRecvHex, row=0, col=0,colspan=1)  #十六进制 清空钮 接收区
    p3.addItem(proxyBtnClrRecv, row=0, col=1,colspan=1)
    p3.addItem(proxyTxtRecv, row=0, col=2,colspan=1)
    p3.addItem(proxyChkBoxSendHex, row=1, col=0,colspan=1)  #十六进制 发送钮 发送区
    p3.addItem(proxyBtnSendMcu, row=1, col=1,colspan=1)     
    p3.addItem(proxyTxtSend, row=1, col=2,colspan=1)
    p3.addItem(proxyBtnSendPc, row=2, col=0,colspan=1)      #PC命令发送钮 发送区
    p3.addItem(proxyTxtCmd, row=2, col=2,colspan=1)
    p3.addItem(proxySta, row=3, col=2, colspan=1)           #Other命令 状态区
    p3.addItem(proxyBtnOther, row=3, col=0,colspan=1)

    #SP，PV，MV，ERR，AMV 数组
    #dataSp=np.zeros(historyLength).__array__('d')     #另一种初始化方式
    dataSp=np.full(historyLength, 0, dtype='int')     #采用 int ，全部填充为0    uint32   d
    dataPv=np.full(historyLength, 0, dtype='int')     
    dataMv=np.full(historyLength, 0, dtype='int')     
    dataErr=np.full(historyLength, 0, dtype='int')    #偏差
    dataAMv=np.full(historyLength, 0, dtype='int')    #控制增量
    #SP，PV，MV，ERR，AMV 曲线
    #curveSp = pxyz.plot(symbol='o',color='y', width=3)     # 绘制一个图形 pen=(i,3)
    curveSp = pxyz.plot(pen=(0,5))     ## setting pen=(i,3) automaticaly creates three different-colored pens
    curvePv = pxyz.plot(pen=(1,5))                                # 绘制一个图形
    curveMv = pxyz.plot(pen=(2,5))                                
    curveErr = pxyz.plot(pen=(3,5))                                
    curveAMv = pxyz.plot(pen=(4,5))                                

    # 中线设置 暂不用
    #dataSpmid=np.full(historyLength, 1, dtype='int')
    #dataPvmid=np.full(historyLength, 63, dtype='int')
    #dataMvmid=np.full(historyLength, 127, dtype='int')
    #dataErrmid=np.full(historyLength, 5000, dtype='int')
    #dataAMvmid=np.full(historyLength, 5000, dtype='int')
    #curveSpmid = pxyz.plot(pen=(0,5))                                # 绘制一个图形
    #curvePvmid = pxyz.plot(pen=(1,5))                                
    #curveMvmid = pxyz.plot(pen=(2,5))                                
    #curveErrmid = pxyz.plot(pen=(3,5))                               
    #curveAMvmid = pxyz.plot(pen=(4,5))                               

    if 1:       #串口初始化
        gCommCnt = len(serial.tools.list_ports.comports())      #获取系统所有串口资源
        for i in range(1,gCommCnt+1):       #编号是从 COM1 开始，逐一尝试打开
            try:
                mSerial = serial.Serial('COM'+str(i), int(115200), timeout=1, parity=serial.PARITY_NONE, stopbits=1)  #等待超时时间单位为s,，即最多等待此时间
                if (mSerial.isOpen()):  
                    print("Open COM{} Success".format(i))
                    #mSerial.write("hello".encode())        # 向端口些数据 字符串必须译码
                    mSerial.flushInput()  # 清空缓冲区
                    btnStop.setText('CLOSE')                # 串口成功，才启动串口进程  注意是通过此文本来确定状态
                    break
            except:
                print('Fail to open COM'+str(i))
        else:                               # 说明所有串口资源都无法打开
                print("Open All Comm Failed")
                QMessageBox.information(win,"错误","没有可用串口，请确认串口号！",QMessageBox.Yes)

        
        th1 = threading.Thread(target=Serial)       #串口进程
        th1.start()    

    #th2 = threading.Thread(target = Keyscan)    #命令进程
    #th2.start()
    timer = pg.QtCore.QTimer()                  #定时刷新
    timer.timeout.connect(plotData)             # 定时刷新数据显示, 这是一个周期性调用的关系，所以数据可以共享
    timer.start(100)  # 多少ms调用一次        原来为1，改为100会不会减少CPU占用率
    app.exec_()
