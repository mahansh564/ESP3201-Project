import rpyc

# Create a RPyC connection to the remote ev3dev device.
# Use the hostname or IP address of the ev3dev device.
# If this fails, verify your IP connectivty via ``ping X.X.X.X``
conn = rpyc.classic.connect('ev3dev.local')
conn.execute("print('Hello Slave. I am your master!')")

motors = conn.modules['ev3dev2.motor'] 
m1 = motors.LargeMotor('outA')
m1.run_timed(speed_sp=100, time_sp=1000)

motors = conn.modules['ev3dev2.motor'] 
tank_drive = motors.MoveTank('outA', 'outD')
tank_drive.on_for_seconds(-40, -40, 1)


m1.stop_action = 'hold'
m1.run_direct(duty_cycle_sp=50) # move m1 up
while True:
    if t1.is_pressed == True:
        m1.position = 0
        m1.reset()
        m1.stop_action = 'hold'
        break