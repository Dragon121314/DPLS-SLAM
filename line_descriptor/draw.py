import matplotlib.pyplot as plt
fig = plt.figure()
# 读取数据
data = []
time=[]
with open('example.txt', 'r') as file:
    for line in file:
        # 将每行的数据转换为整数并添加到列表中
        parts = line.strip().split(' ')
        data.append(float(parts[0]))  # 将第一列的数据添加到列表中
        time.append(1000*float(parts[1]))  # 将第一列的数据添加到列表中

ax1 = fig.add_subplot(2, 1, 1)  # 参数分别代表：行数、列数、子图编号
ax1.plot(data)  # 在第一个子图中绘制数据
# ax1.text(0.5, 0.01, 'Title at the Bottom', ha='center', fontsize=12,transform=ax1.transAxes)
ax1.set_xlabel('the Gradient Threshold',fontsize=12)
ax1.set_ylabel('the Length Factor',fontsize=12)
ax1.xaxis.label.set_weight('bold') 
ax1.yaxis.label.set_weight('bold') 
ax1.grid(True)



ax2 = fig.add_subplot(2, 1, 2)  # 参数分别代表：行数、列数、子图编号
ax2.plot(time)
ax2.set_xlabel('the Gradient Threshold',fontsize=12)
ax2.set_ylabel('the Time [ms]',fontsize=12)
ax2.xaxis.label.set_weight('bold') 
ax2.yaxis.label.set_weight('bold') 
ax2.grid(True)
# ax2.text(0.5, 0.01, 'Title at the Bottom', ha='center', fontsize=12,transform=ax2.transAxes)

# 绘制图形
# plt.plot(data)  # 使用plot函数绘制折线图
# plt.title('Data from Text File')  # 设置图表标题
# plt.xlabel('Index')  # 设置x轴标签
# plt.ylabel('Value')  # 设置y轴标签

plt.tight_layout() 
plt.show()  # 显示图形







