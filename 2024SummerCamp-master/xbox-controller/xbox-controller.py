import socket
import pygame
import time
import win32gui
import win32con

server_ip = "192.168.111.111"
server_port = 8000

# 摇杆倍率
slow_k = 150
fast_k = 400

# 创建一个UDP套接字
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# 接收方的地址和端口
udp_addr = (server_ip, server_port)

# 初始化PyGame
pygame.init()

# 读取USB控制器的输入
joystick_count = pygame.joystick.get_count()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# 设置游戏窗口
screen = pygame.display.set_mode((300, 50))
pygame.display.set_caption("请让该窗口保持聚焦")
screen.fill((255,255,255))
# 创建一个时钟对象，用于控制游戏的更新速度
clock = pygame.time.Clock()

# 前置保持
hwnd = pygame.display.get_wm_info()['window']

# 左右位移球标签
LEFT = 1
RIGHT = 2
run_ball_flag = 0

def send(data):
    # 发送数据到指定的ip和端口
    udp_socket.sendto(data.encode(), udp_addr)
    time.sleep(0.01)

if __name__ == '__main__':

    # 主循环
    running = True
    while running:
        # 处理事件
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.ACTIVEEVENT:
                if event.state == 2 and event.gain == 0:
                    # 窗口前置
                    win32gui.SetWindowPos(hwnd, win32con.HWND_TOPMOST, 0, 0, 0, 0, win32con.SWP_NOMOVE | win32con.SWP_NOSIZE)
                    time.sleep(0.01)
                    # 停机
                    # send("xA")
                    # send("l0;0l")
                    # send("r0;0r")

        # 先读取扳机判断是否进入加速模式
        speed_k = (joystick.get_axis(5) + 1) / 2  # RT
        control_k = slow_k + speed_k*(fast_k-slow_k)

        # 读取摇杆
        move_row = int(joystick.get_axis(0) * control_k)       # 左手横轴
        move_pitch = int(joystick.get_axis(1) * control_k)     # 左手纵轴
        round_row = int(joystick.get_axis(2) * control_k)        # 右手横轴
        round_pitch = int(joystick.get_axis(3) * control_k)      # 右手纵轴

        # if move_row < 10:
        #     move_row = 0
        # if move_pitch < 10:
        #     move_pitch = 0
        # if round_row < 10:
        #     round_row = 0
        # if round_pitch < 10:
        #     round_pitch = 0

        # # 读取按钮
        # buttom_num = joystick.get_numbuttons()
        # for i in range(buttom_num):
        #     if (joystick.get_button(i)):
        #         print(i)

        #十字按键
        if joystick.get_hat(0)[1] == 1: #上
            send("xU")
        if joystick.get_hat(0)[1] == -1: #下
            send("xD")
        if joystick.get_hat(0)[0] == -1: #左
            send("xL")
        if joystick.get_hat(0)[0] == 1: #右
            send("xR")

        # 摇杆按钮
        if (joystick.get_button(0)):  # A
            send("xA")
        if (joystick.get_button(1)):  # B
            send("xB")
        if (joystick.get_button(2)):  # X
            send("xX")
        if (joystick.get_button(3)):  # Y
            send("xY")
        if (joystick.get_button(4)):  # LB
            send("x<")
        if (joystick.get_button(5)):  # RB
            send("x>")
        if (joystick.get_button(6)):  # -
            send("x-")
            time.sleep(0.5)
        if (joystick.get_button(7)):  # +
            send("x+")
            time.sleep(0.5)

        # 读取键盘
        keys_pressed = pygame.key.get_pressed()
        if keys_pressed[pygame.K_1]:
            send("k1")
        if keys_pressed[pygame.K_2]:
            send("k2")
        if keys_pressed[pygame.K_3]:
            send("k3")
        if keys_pressed[pygame.K_q]:
            send("kQ")
        if keys_pressed[pygame.K_w]:
            send("kW")
        if keys_pressed[pygame.K_e]:
            send("kE")
        if keys_pressed[pygame.K_r]:
            send("kR")
        if keys_pressed[pygame.K_t]:
            send("kT")
        if keys_pressed[pygame.K_y]:
            send("kY")
        if keys_pressed[pygame.K_a]:
            send("kA")
        if keys_pressed[pygame.K_s]:
            send("kS")
        if keys_pressed[pygame.K_d]:
            send("kD")
        if keys_pressed[pygame.K_f]:
            send("kF")
        if keys_pressed[pygame.K_g]:
            send("kG")
        if keys_pressed[pygame.K_h]:
            send("kH")


        send("l{round_row};{round_pitch}l".format(round_row=round_row,round_pitch=round_pitch))    #  "l" 标志代表旋转摇杆
        send("r{move_row};{move_pitch}r\n".format(move_row=move_row,move_pitch=move_pitch))   #  "r" 标志代表平移摇杆
        # print("l{round_row};{round_pitch}l".format(round_row=round_row,round_pitch=round_pitch))
        # print("r{move_row};{move_pitch}r\n".format(move_row=move_row,move_pitch=move_pitch))

        # 更新游戏画面
        pygame.display.flip()
        # 控制游戏帧率
        clock.tick(2000)

    # 退出PyGame
    pygame.quit()