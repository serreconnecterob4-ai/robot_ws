
## Paramètre du robot

### Vitesse maximum : 

robot_ws/src/curt_mini/curt_mini/config/ros2_control_simulation.yaml ---- Lignes 53 - 65

````py
    linear.x.has_velocity_limits: true
    linear.x.min_velocity: -1.5
    linear.x.max_velocity: 1.5
    linear.x.has_acceleration_limits: true
    linear.x.min_acceleration: -2.5
    linear.x.max_acceleration: 2.5

    angular.z.has_velocity_limits: true
    angular.z.min_velocity: -12.0
    angular.z.max_velocity: 12.0
    angular.z.has_acceleration_limits: true
    angular.z.min_acceleration: -15.0
    angular.z.max_acceleration: 15.0
````

### Position du robot initiale :