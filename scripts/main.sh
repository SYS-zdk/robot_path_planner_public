source ../devel/setup.bash
# For simulation runs, a leftover Gazebo instance is the #1 reason of:
# - /gazebo services already existing
# - model name collision ("already exist")
# - spawn pose seemingly not taking effect
FORCE_KILL_GAZEBO=1 bash "$(dirname "$0")/clear_tf_cache.sh"

# Public repo cleanup note:
# The original workspace used a local dynamic launch/config generator (dynamic_xml_config).
# In the public version we keep a static, checked-in launch entry.
roslaunch sim_env main.launch
