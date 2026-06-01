# PID controller class
# Copyright 2024, Prof. Hamid Ossareh @ University of Vermont 
class PID:
    # constructor
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, N=0.1, Ts=1/60, initialState=0.0, umax=1, umin=-1, Kaw=0):
        self.istate = initialState    # initial condition of the integrator
        self.dstate = 0               # initial condition of the derivative filter
        self.actuatorError_prev = 0   # holds previous value of (v-u)
        self.output = initialState    # initial controller output
        self.error_prev = 0           # previous value of the error signal

        # set the PID parameters
        self.Kp, self.Ki, self.Kd, self.N, self.Kaw = Kp, Ki, Kd, N, Kaw
        self.Ts = Ts
        self.umin, self.umax = umin, umax



    def update(self, setpoint, output):
        # tracking error
        error = setpoint - output

        # update the integral state, apply antiwindup as needed
        integratorInput = self.Ki*error + self.Kaw*(self.actuatorError_prev)
        self.istate += self.Ts*integratorInput
        
        # Calculate filtered derivative
        self.dstate = 1/self.N*(error - self.error_prev - (self.Ts-self.N)*self.dstate)
        self.error_prev = error
 
        # Calculate command using PID equation
        command = self.Kp*error + self.istate + self.Kd*self.dstate

        # Saturate command
        if command > self.umax:
            self.output = self.umax
        elif command < self.umin:
            self.output = self.umin
        else:
            self.output = command

        # Store previous saturated command
        self.actuatorError_prev = self.output-command

        return self.output