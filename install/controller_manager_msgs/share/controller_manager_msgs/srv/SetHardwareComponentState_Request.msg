# The SetHardwareComponentState service allows to control life-cycle of a single hardware component.
# Supported states are defined in the design document of LifecycleNodes available at:
# https://design.ros2.org/articles/node_lifecycle.html
# To control life-cycle of a hardware component, specify its "name" and "target_state".
# Target state may be defined by "id" using a constant from `lifecycle_msgs/msg/State` or a label
# using definitions from `hardware_interface/types/lifecycle_state_names.hpp` file.
# The return value "ok" indicates if the component has successfully changed its state to "target_state".
# The return value "state" returns current state of the hardware component.

string name
lifecycle_msgs/State target_state
