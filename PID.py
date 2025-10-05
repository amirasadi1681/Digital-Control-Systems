from typing import Any

class PID:
    def __init__(self, k:float, k_i:float, k_d:float, T, Derivative_LP_Filter_in_percent:int = 0) -> None:
        self.k = k
        self.k_i = k_i
        self.k_d = k_d
        self.T = T
        if Derivative_LP_Filter_in_percent > 100 or Derivative_LP_Filter_in_percent < 0:
            raise ValueError
        self.N = Derivative_LP_Filter_in_percent/100.0
        self.previous_error:float = 0
        self.previous_Derivative:float = 0
        self.previous_Integral:float = 0
    
    def __call__(self, error) -> float:
        proportional = self.k*error # proportional part
        # print("proportional:", proportional)
        Integral = self.k_i *(self.previous_Integral + (error * self.T)) # self.previous_Integral + (error * self.T * self.k_i) # integral part
        # print("integral:", Integral)
        Derivative = self.k_d*((error - self.previous_error)/self.T) #self.k_d*(self.N/(1+(self.N * self.T)))*(error - self.previous_error)  + self.previous_Derivative # derivatives part
        # output command
        u = proportional + Integral + Derivative
        # update for next iterations
        self.previous_error = error
        self.previous_Integral = Integral
        self.previous_Derivative = Derivative
        return u
    
    
