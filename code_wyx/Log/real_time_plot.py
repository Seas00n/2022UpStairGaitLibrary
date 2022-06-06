import pyqtgraph as pg
import time
import numpy as np
import serial


def plot_all():
    start = time.time()
    q_thigh_list.append(data_all[0])
    acc_list.append(data_all[1])
    phase_list.append(data_all[2])
    state_list.append(data_all[3] * 50)
    threshold_list.append(data_all[4])
    L_list.append(data_all[5]/10)
    q_desired_list.append(data_all[6])
    kp_list.append(data_all[7])
    tau_list.append(data_all[8])
    threshold_list2.append(data_all[9])
    if len(q_thigh_list) == 100:
        q_thigh_list.pop(0)
        acc_list.pop(0)
        phase_list.pop(0)
        state_list.pop(0)
        threshold_list.pop(0)
        L_list.pop(0)
        q_desired_list.pop(0)
        kp_list.pop(0)
        tau_list.pop(0)
        threshold_list2.pop(0)
    plot1.setData(q_thigh_list, pen=pg.mkPen(color='g', width=5))
    curve_threshold1.setData(np.array(threshold_list))
    curve_threshold2.setData(np.array(threshold_list2))
    plot2.setData(acc_list, pen=pg.mkPen(color='b', width=5))
    plot3.setData(phase_list, pen=pg.mkPen(color='r', width=5))
    curve_state.setData(np.array(state_list))
    plot4.setData(L_list, pen=pg.mkPen(color='r', width=5))
    #curve_stiffness.setData(np.array(kp_list))
    curve_angle.setData(np.array(q_desired_list))
    end = time.time()
    print("cost:", end - start)


if __name__ == '__main__':
    # [大腿角度，加速度，相位，状态，切换阈值，期望腿长，期望踝角度 / 期望膝角度，期望刚度 / 阻尼 / 平衡角度，期望力矩，其他]
    q_thigh_list = [0]
    acc_list = [0]
    phase_list = [0]
    state_list = [2]
    threshold_list = [0]
    threshold_list2 = [0]
    L_list = [0]
    q_desired_list = [0]
    kp_list = [0]
    tau_list = [0]
    curve_threshold1 = pg.PlotCurveItem(pen=({'color': 'r', 'width': 3, 'symbol': '*'}), skipFiniteCheck=True)
    curve_threshold2 = pg.PlotCurveItem(pen=({'color': 'r', 'width': 3, 'symbol': '*'}), skipFiniteCheck=True)
    curve_state = pg.PlotCurveItem(pen=({'color': 'c', 'width': 4, 'symbol': '*'}), skipFiniteCheck=True)
    curve_stiffness = pg.PlotCurveItem(pen=({'color': 'c', 'width': 4, 'symbol': '*'}), skipFiniteCheck=True)
    curve_tau = pg.PlotCurveItem(pen=({'color': 'c', 'width': 4, 'symbol': '*'}), skipFiniteCheck=True)
    curve_angle = pg.PlotCurveItem(pen=({'color': 'c', 'width': 4, 'symbol': '*'}), skipFiniteCheck=True)
    app = pg.mkQApp()  # 建立app
    pg.setConfigOption('background', 'white')
    pg.setConfigOption('foreground', 'k')
    win = pg.GraphicsWindow()  # 建立窗口
    win.setWindowTitle('波形图')
    win.resize(800, 500)  # 小窗口大小
    # 创建图表
    historyLength = 100  # 横坐标长度
    p1 = win.addPlot()  # 把图p加入到窗口中
    p1.showGrid(x=True, y=True)  # 把X和Y的表格打开
    p1.setRange(xRange=[0, historyLength], yRange=[-60, 60], padding=0)  # x轴和y轴的范围
    p1.setLabel(axis='left', text='Thigh_Angle')  # 靠左
    p1.setLabel(axis='bottom', text='时间')
    p1.setTitle('大腿角度')  # 表格的名字
    p1.addItem(curve_threshold1)
    p1.addItem(curve_threshold2)
    plot1 = p1.plot()

    p2 = win.addPlot()  # 把图p加入到窗口中
    p2.showGrid(x=True, y=True)  # 把X和Y的表格打开
    p2.setRange(xRange=[0, historyLength], yRange=[-50, 50], padding=0)  # x轴和y轴的范围
    p2.setLabel(axis='left', text='Accel_z')  # 靠左
    p2.setLabel(axis='bottom', text='时间')
    p2.setTitle('加速度')  # 表格的名字
    plot2 = p2.plot()

    win.nextRow()
    p3 = win.addPlot()  # 把图p加入到窗口中
    p3.showGrid(x=True, y=True)  # 把X和Y的表格打开
    p3.setRange(xRange=[0, historyLength], yRange=[-10, 100], padding=0)  # x轴和y轴的范围
    p3.setLabel(axis='left', text='Phase')  # 靠左
    p3.setLabel(axis='bottom', text='时间')
    p3.setTitle('相位')  # 表格的名字
    plot3 = p3.plot()
    p3.addItem(curve_state)

    p4 = win.addPlot()
    p4.showGrid(x=True, y=True)
    p4.setRange(xRange=[0, historyLength], yRange=[-10, 90], padding=0)  # x轴和y轴的范围
    p4.setLabel(axis='left', text='Phase')  # 靠左
    p4.setLabel(axis='bottom', text='时间')
    p4.setTitle('腿长')  # 表格的名字
    plot4 = p4.plot()
    p4.addItem(curve_angle)

    data_all = np.memmap('plot_buf.npy', dtype='float32', mode='r', shape=(10,))
    # 设置定时器
    timer = pg.QtCore.QTimer()
    timer.timeout.connect(plot_all)  # 定时刷新数据显示
    timer.start(40)  # 多少ms调用一次

    app.exec_()
