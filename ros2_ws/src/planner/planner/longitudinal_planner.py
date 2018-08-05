import rclpy
from rclpy.node import Node
import cvxpy as cvx
from util import util
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from opencaret_msgs.msg import LeadVehicle, LongitudinalPlan
import time
import numpy as np
INITIAL_CRUISING_SPEED = 15.0
INITIAL_DISTANCE_TO_LEAD_CAR = 100.0


class LongitudinalPlanner(Node):

    def __init__(self):
        super(LongitudinalPlanner, self).__init__('longitudinal_planner')
        self.T = 20
        self.dt = 0.1
        self.min_follow_distance = cvx.Parameter(value=5.0)
        self.max_acceleration = cvx.Parameter(value=3.0)
        self.min_acceleration = cvx.Parameter(value=-3.0)
        self.min_max_jerk = cvx.Parameter(value=2.0)

        self.cruising_speed = cvx.Parameter(value=util.mph_to_ms(INITIAL_CRUISING_SPEED))
        self.last_v_trajectory = None
        self.last_a_trajectory = None
        self.last_x_trajectory = None
        self.v_ego = cvx.Parameter(value=0.0)
        self.a_ego = cvx.Parameter(value=0.0)
        self.x_lead = cvx.Parameter(value=INITIAL_DISTANCE_TO_LEAD_CAR)
        self.v_lead = cvx.Parameter(value=self.cruising_speed.value)
        self.a_lead = cvx.Parameter(value=0.0)

        self.v = cvx.Variable(self.T + 1)
        self.a = cvx.Variable(self.T)
        self.j = cvx.Variable(self.T)
        self.x = cvx.Variable(self.T + 1)

        self.solver = self.init_mpc_solver()
        self.cruising_speed_sub = self.create_subscription(Float32, 'cruising_speed', self.on_cruising_speed)
        self.speed_sub = self.create_subscription(Float32, 'wheel_speed', self.on_wheel_speed)
        self.lead_car_sub = self.create_subscription(LeadVehicle, 'lead_vehicle', self.on_lead_vehicle)
        self.imu_sub = self.create_subscription(Imu, 'imu', self.on_imu)
        self.computed_accel_sub = self.create_subscription(Float32, 'computed_accel', self.on_computed_accel)

        self.plan_pub = self.create_publisher(LongitudinalPlan, 'longitudinal_plan')
        self.timer = self.create_timer(1.0 / 20, self.make_plan)

    def on_cruising_speed(self, msg):
        self.cruising_speed.value = msg.data

    def on_wheel_speed(self, msg):
        self.v_ego.value = util.mph_to_ms(msg.data)

    def on_computed_accel(self, msg):
        self.a_ego.value = msg.data

    def on_lead_vehicle(self, msg):
        self.x_lead.value = msg.distance
        self.v_lead.value= msg.velocity
        self.a_lead.value = msg.accel

    def on_imu(self, msg):
        self.a_ego.value = msg.linear_acceleration.x

    def init_mpc_solver(self):
        states = []
        for t in range(self.T):
            cost = cvx.sum_squares(self.v[t + 1] - self.cruising_speed) + cvx.sum_squares(self.j[t]) * 5 + \
                   cvx.sum_squares((self.min_follow_distance + 3) - self.x[t + 1])
            # if t > 0:
            #     cost += cvx.sum_squares(a[t] - a[t - 1]) * 10

            #  Setup constraints for a basic kinematic model of the car assuming constant velocity
            #  of the lead car for the time being

            constr = [self.v[t + 1] == self.v[t] + self.a[t] * self.dt,
                      self.x[t + 1] == self.x[t] - self.v[t] * self.dt - self.a[t] * self.dt * self.dt / 2. + self.v_lead * self.dt,
                      self.x[t + 1] >= self.min_follow_distance,
                      self.v[t + 1] <= self.cruising_speed,
                      self.a[t] <= self.max_acceleration,
                      self.a[t] >= self.min_acceleration,
                      self.v[t + 1] >= 0.0
                      ]

            if t > 0:
                constr += [self.j[t] == (self.a[t] - self.a[t - 1]) / self.dt,
                           self.j[t] <= self.min_max_jerk,
                           self.j[t] >= -self.min_max_jerk]

            prob = cvx.Problem(cvx.Minimize(cost), constr)
            if not prob.is_dcp():
                print("not dcp at t = {}".format(t))
                exit()
            states.append(prob)
        # sums problem objectives and concatenates constraints.
        prob = sum(states)
        constraints = [self.v[0] == self.v_ego,
                       self.a[0] == self.a_ego,
                       self.j[0] == 0.0,
                       self.x[0] == self.x_lead] + prob.constraints
        return cvx.Problem(prob.objective, constraints=constraints)

    def make_plan(self):

        start = time.time()
        try:
            self.solver.solve()
        except Exception as e:
            print(e)


        # print("time taken: {}, Solve time: {}, Setup Time: {}".format(time.time() - start,
        #                                                               self.solver.solver_stats.solve_time,
        #                                                               self.solver.solver_stats.setup_time))
        self.last_v_trajectory = self.x.value
        self.last_a_trajectory = self.a.value
        self.last_x_trajectory = self.x.value
        plan = LongitudinalPlan()
        plan.dt = [float(t * self.dt) for t in range(self.T)]
        # print(util.ms_to_mph(np.array(self.v.value).reshape(-1)[5]))
        plan.distance_from_lead = np.array(self.x.value).reshape(-1).tolist()
        plan.accel = np.array(self.a.value).reshape(-1).tolist()
        plan.velocity = np.array(self.v.value).reshape(-1).tolist()
        self.plan_pub.publish(plan)

def main():
    rclpy.init()
    planner = LongitudinalPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()