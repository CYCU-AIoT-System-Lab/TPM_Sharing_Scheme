class Traffic:
    def __init__(self, window_count, window_size):
        self.window_count = window_count
        self.window_size = window_size
        self.p_win = 0
        self.vec = [0] * window_count
        self.old_f2 = 0
        self.previous_time = 0
        print(f"window_count {self.window_count} window_size {self.window_size}")
        print(self.vec)

    def cal_F2(self):
        f2 = 0
        for i in range(self.window_count):
            f2 += self.vec[i] * self.vec[i]
        return f2

    def compare_F2(self, f2):
        if abs(f2 - self.old_f2) > 10:
            self.old_f2 = f2
            print("F2 changed too much")

    # note: dividion and modular operation can be optimized by bit operation
    def traffic(self, current_time):
        elapse_time = current_time - self.previous_time
        if elapse_time >= self.window_size:
            print("window switch")
            p_inc = int(elapse_time / self.window_size) # opt
            self.p_win += p_inc
            self.previous_time += p_inc * self.window_size
            if self.p_win >= self.window_count:
                f2 = self.cal_F2()
                print(f"interval switch, f2 = {f2}")
                self.compare_F2(f2)
                self.p_win %= self.window_count # opt
                self.vec = [0] * self.window_count
        self.vec[self.p_win] += 1
        print(f"{self.vec} p_win {self.p_win} previous_time {self.previous_time} current_time {current_time} elapse_time {elapse_time}")

if __name__ == "__main__":
    traffic = Traffic(4, 10)
    old_temp = 0
    while True:
        temp = int(input("Enter time: "))
        if temp < old_temp:
            print("Input should never be less than previous input")
        else:
            traffic.traffic(temp)
            old_temp = temp
