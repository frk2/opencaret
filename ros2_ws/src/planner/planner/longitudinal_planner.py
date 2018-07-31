import rclpy
from rclpy.node import Node
import cvxpy as cvx
import util
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from opencaret_msgs.msg import LeadVehicle, LongitudinalPlan

INITIAL_CRUISING_SPEED = 35.0
INITIAL_DISTANCE_TO_LEAD_CAR = 100.0

class LongitudinalPlanner(Node):

    def __init__(self):
        super(LongitudinalPlanner, self).__init__('longitudinal_planner')
        self.T = 20
        self.dt = 0.2
        self.min_follow_distance = 5.0
        self.max_acceleration = 5.0
        self.min_acceleration = -5.0
        self.min_max_jerk = 3.0

        self.cruising_speed = util.mph_to_ms(INITIAL_CRUISING_SPEED)
        self.last_v_trajectory = None
        self.last_a_trajectory = None
        self.last_x_trajectory = None
        self.v_ego = 0
        self.a_ego = 0
        self.x_lead = INITIAL_DISTANCE_TO_LEAD_CAR
        self.v_lead = self.cruising_speed
        self.a_lead = 0
        self.init_mpc_solver()

        self.cruising_speed_sub = self.create_subscription(Float32, 'cruising_speed', self.on_cruising_speed)
        self.speed_sub = self.create_subscription(Float32, 'wheel_speed', self.on_wheel_speed)
        self.lead_car_sub = self.create_subscription(LeadVehicle, 'lead_vehicle', self.on_lead_vehicle)
        self.imu_sub = self.create_subscription(Imu, 'imu', self.on_imu)

        self.plan_pub = self.create_publisher(LongitudinalPlan, 'longitudinal_plan')
        self.timer = self.create_timer(1.0 / 5, self.make_plan)

    def on_cruising_speed(self, msg):
        self.cruising_speed = msg.data
        
    def on_wheel_speed(self, msg):
        self.v_ego = msg.data

    def on_lead_vehicle(self, msg):
        self.x_lead = msg.distance
        self.v_lead = msg.velocity
        self.a_lead = msg.accel

    def on_imu(self, msg):
        self.a_ego = msg.linear_acceleration.x

    def make_plan(self):
        v = cvx.Variable(self.T + 1)
        a = cvx.Variable(self.T)
        j = cvx.Variable(self.T)
        x = cvx.Variable(self.T + 1)

        states = []
        for t in range(self.T):
            cost = cvx.sum_squares(v[t + 1] - self.cruising_speed) + cvx.sum_squares(j[t]) * 3 + \
                   cvx.sum_squares(x[t + 1])
            if t > 0:
                cost += cvx.sum_squares(a[t] - a[t - 1]) * 10

            #  Setup constraints to
            constr = [v[t + 1] == v[t] + a[t] * self.dt,
                      x[t + 1] == x[t] - v[t] * self.dt - a[t] ** self.dt / 2. + self.v_lead * self.dt,
                      x[t + 1] >= self.min_follow_distance,
                      v[t + 1] <= self.cruising_speed,
                      a[t] <= self.max_acceleration,
                      a[t] >= self.min_acceleration,
                      v[t + 1] >= 0.0
                      ]

            if t > 0:
                constr += [j[t] == (a[t] - a[t - 1]) / self.dt,
                           j[t] <= self.min_max_jerk,
                           j[t] >= -self.min_max_jerk]

            states.append(cvx.Problem(cvx.Minimize(cost), constr))
        # sums problem objectives and concatenates constraints.
        prob = sum(states)
        constraints = [v[0] == self.v_ego,
                       a[0] == self.a_ego,
                       j[0] == 0.0,
                       x[0] == self.x_lead] + prob.constraints
        prob2 = cvx.Problem(prob.objective, constraints=constraints)
        prob2.solve()
        self.last_v_trajectory = v.value
        self.last_a_trajectory = a.value
        self.last_x_trajectory = x.value
        plan = LongitudinalPlan()
        plan.dt = [x for t in range(self.T)]
        plan.distance_from_lead = x.value
        plan.accel = a.value
        plan.velocity = v.value
        self.plan_pub.publish(plan)

def main():
    rclpy.init()
    planner = LongitudinalPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
