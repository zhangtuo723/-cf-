class pid(object):
    def __init__(self,exp_val,p,i,d):
        self.kp=p
        self.ki=i
        self.kd=d
        self.now_err=0 #现在误差
        self.last_err=0#上一次误差
        self.now_val=0#现在值
        self.sum_err=0#累计误差
    def cmd_pid(self,sub_s):
        """位置式PID控制"""
        self.last_err=self.now_err
        self.now_err=sub_s-self.now_val
        self.sum_err+=self.now_err
        self.now_val=self.kp*self.now_err + self.ki * self.sum_err + self.kd* (self.now_err-self.last_err)
        # print(self.now_val)
        return self.now_val
