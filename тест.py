import math
import matplotlib.pyplot as plt

def pid_control(x_c, y_c, theta_c, x_g, y_g, dt):
    # Параметры робота
    r = 0.0325  # Радиус колеса (м)
    L = 0.191   # Расстояние между колесами (м)
    v_max = 1.0  # Макс. линейная скорость (м/с)
    omega_max = 2.0  # Макс. угловая скорость (рад/с)
    epsilon = 0.01  # Допустимая ошибка (м)

    # Коэффициенты ПИД для линейной скорости
    K_p_rho = 0.5
    K_i_rho = 0.01
    K_d_rho = 0.1

    # Коэффициенты ПИД для угловой скорости
    K_p_beta = 2.0
    K_i_beta = 0.05
    K_d_beta = 0.2

    # Статические переменные для интеграла и предыдущих ошибок
    global integral_rho, integral_beta, prev_rho, prev_beta
    if not hasattr(pid_control, "integral_rho"):
        integral_rho = 0.0
        integral_beta = 0.0
        prev_rho = 0.0
        prev_beta = 0.0

    # 1. Вычисление ошибки по расстоянию
    rho = math.sqrt((x_g - x_c)**2 + (y_g - y_c)**2)
    if rho < epsilon:
        return [0, 0, 0, 0, x_c, y_c, theta_c, rho]  # Остановка

    # 2. Вычисление угловой ошибки
    alpha = math.atan2(y_g - y_c, x_g - x_c)
    beta = alpha - theta_c
    beta = math.atan2(math.sin(beta), math.cos(beta))

    # 3. ПИД-регулятор для линейной скорости
    integral_rho += rho * dt
    derivative_rho = (rho - prev_rho) / dt if dt > 0 else 0
    v = K_p_rho * rho + K_i_rho * integral_rho + K_d_rho * derivative_rho
    v = min(max(v, -v_max), v_max)  # Ограничение скорости

    # 4. ПИД-регулятор для угловой скорости
    integral_beta += beta * dt
    derivative_beta = (beta - prev_beta) / dt if dt > 0 else 0
    omega = K_p_beta * beta + K_i_beta * integral_beta + K_d_beta * derivative_beta
    omega = min(max(omega, -omega_max), omega_max)  # Ограничение угловой скорости

    # 5. Сохранение ошибок для следующего шага
    prev_rho = rho
    prev_beta = beta

    # 6. Вычисление скоростей колес
    v_R = v + (omega * L) / 2
    v_L = v - (omega * L) / 2
    omega_R = v_R / r
    omega_L = v_L / r

    # 7. Назначение скоростей колес
    omega_1 = omega_3 = omega_L
    omega_2 = omega_4 = omega_R

    # 8. Обновление состояния робота
    x_c += v * math.cos(theta_c) * dt
    y_c += v * math.sin(theta_c) * dt
    theta_c += omega * dt

    return [omega_1, omega_2, omega_3, omega_4, x_c, y_c, theta_c, rho]

def simulate_robot():
    # Начальные условия
    x_c, y_c, theta_c = 0.0, 0.0, 0.0
    x_g, y_g = 10.0, 10.0
    dt = 0.01  # Временной шаг (с)
    trajectory = [(x_c, y_c)]  # Список для хранения траектории
    times = [0.0]  # Список для хранения времени
    errors = [math.sqrt((x_g - x_c)**2 + (y_g - y_c)**2)]  # Список для хранения погрешностей
    time = 0.0

    # Симуляция
    while True:
        result = pid_control(x_c, y_c, theta_c, x_g, y_g, dt)
        omega_1, omega_2, omega_3, omega_4, x_c, y_c, theta_c, rho = result
        trajectory.append((x_c, y_c))
        time += dt
        times.append(time)
        errors.append(rho)

        if rho < 0.01:  # Условие достижения цели
            print(f"Goal reached at time: {time:.2f} seconds")
            print(f"Final position: ({x_c:.2f}, {y_c:.2f}), orientation: {theta_c:.2f} rad")
            print(f"Final error: {rho:.4f} m")
            break

    # Визуализация траектории
    plt.figure(figsize=(10, 6))
    # plt.subplot(1, 2, 1)
    # x_coords, y_coords = zip(*trajectory)
    # plt.plot(x_coords, y_coords, 'b-', label='Trajectory')
    # plt.plot(x_g, y_g, 'ro', label='Goal')
    # plt.plot(0, 0, 'go', label='Start')
    # plt.xlabel('x (m)')
    # plt.ylabel('y (m)')
    # plt.title('Robot Trajectory')
    # plt.legend()
    # plt.grid(True)
    # plt.axis('equal')

    # Визуализация погрешности от времени
    # plt.subplot(1, 2, 2)
    plt.plot(times, errors, 'r-', label='Error (ρ)')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m)')
    plt.title('Error vs Time')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    simulate_robot()