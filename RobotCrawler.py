class RoboCrawler:
    def __init__(self, conn):
        # Use conn.modules for EV3 modules and devices
        #self.ev3 = conn.modules['ev3dev2.ev3brick']
        self.conn = conn
        sound = conn.modules['ev3dev2.sound']
        self.sound = sound.Sound()
        motors = conn.modules['ev3dev2.motor']
        sensors = conn.modules['ev3dev2.sensor.lego']
        
        # Initialize motors and sensors
        self.ax11 = motors.LargeMotor('outA')
        self.ax12 = motors.LargeMotor('outD')
        self.ax2 = motors.MediumMotor('outB')
        self.endstop_ax1 = sensors.TouchSensor('in4')
        self.ultrasonic = sensors.UltrasonicSensor('in3')
        self.gyro = sensors.GyroSensor('in2')


        self.Homing_Speed = 80
        self.ax1_speed = self.Homing_Speed
        self.ax2_speed = self.Homing_Speed * 8

        self.ax1_angle_limits = [0, 100]
        self.ax2_angle_limits = [0, 4500]

    def TestSensor(self, sensor, seconds):
        watch = self.conn.modules['ev3dev2.watch'].StopWatch()
        watch.reset()

        while watch.time() < seconds * 1000:
            self.conn.modules['time'].sleep(0.1)
            if sensor == "endstop1":
                print("endstop1 state is:", self.endstop_ax1.is_pressed)
            elif sensor == "motor11":
                print("Motor11 encoder state is:", self.ax11.position)
            elif sensor == "motor12":
                print("Motor12 encoder state is:", self.ax12.position)
            elif sensor == "motor2":
                print("Motor2 encoder state is:", self.ax2.position)
            elif sensor == "ultrasonic":
                print("Ultrasonic distance:", self.ultrasonic.distance_centimeters)
            elif sensor == "gyro":
                print("Gyro angle:", self.gyro.angle)
            else:
                print("Unknown sensor")

    def Homing(self):
        # Axis 1
        while True:
            self.ax11.run_forever(speed_sp=-self.Homing_Speed)
            self.ax12.run_forever(speed_sp=-self.Homing_Speed)
            if self.endstop_ax1.is_pressed:
                self.ax11.stop(stop_action="hold")
                self.ax12.stop(stop_action="hold")
                self.ax11.position = 0
                self.ax12.position = 0
                self.sound.beep()
                break

        # Axis 2
        while True:
            self.ax2.run_forever(speed_sp=-self.Homing_Speed * 10)
            if self.ax2.is_stalled:
                self.ax2.stop(stop_action="hold")
                break
        
        self.ax2.position = 0
        self.sound.beep()

    def go_to_angle(self, angle1=-1, angle2=-1):
        if angle1 < 0:
            angle1 = self.ax11.position
            
        if angle2 < 0:
            angle2 = self.ax2.position

        self.ax11.run_to_abs_pos(position_sp=angle1, speed_sp=self.ax1_speed, stop_action="hold")
        self.ax12.run_to_abs_pos(position_sp=angle1, speed_sp=self.ax1_speed, stop_action="hold")
        self.ax12.wait_until_not_moving()
        self.ax2.run_to_abs_pos(position_sp=angle2, speed_sp=self.ax2_speed, stop_action="hold")
        self.ax2.wait_until_not_moving()

    def Walk(self, steps):
        for _ in range(steps):
            self.go_to_angle(70, 4000)
            self.go_to_angle(30, 1000)

    def distance_mm(self):
        return self.ultrasonic.distance_centimeters * 10

    def beep(self, n):
        for _ in range(n):
            self.conn.modules['time'].sleep(0.3)
            self.sound.beep()

    def getState(self):
        return self.ax11.position, self.ax2.position