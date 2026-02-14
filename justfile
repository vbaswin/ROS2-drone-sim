preset := "debug"
# hello
executable := "build/drone_sim"
teleop_exe := "build/drone_teleop"

# Status indicatorsj
SUCCESS := "✅"
NOTICE  := "ℹ️"
CAUTION := "⚠️"  
FAILURE := "❌"



# Developoment lifecycle

INIT  := "⚙️"
BUILD   := "🔨"
LINK    := "🔗"
RUN     := "▶️"
CLEAN   := "🧹" 
DRONE   := "🚁"
BRIDGE  := "🌉"


default: run
init: configure setup

configure: 
    @if [ -z "$$ROS_DISTRO" ]; then echo {{FAILURE}} "Error: ROS2 is not sourced." && exit 1; fi
    @command -v ccache > /dev/null || echo "{{CAUTION}} ccache not found, builds may be slower"
    @command -v lld > /dev/null || echo "{{CAUTION}} lld not found, using default linker"
    @echo {{NOTICE}} " Configuring CMake with preset {{preset}}..."
    @cmake --preset {{preset}}

build: 
    @echo {{BUILD}} " Building project..."
    @cmake --build --preset {{preset}}

run: build 
    @echo {{RUN}} " Running Application..."
    @./{{executable}}

teleop: build
    @echo "{{DRONE}} Running Drone Teleop (vim-bindings) ..."
    @./{{teleop_exe}}

gazebo:
    @echo "{{DRONE}} Launching Gazebo simulation..."
    @ign gazebo worlds/multi_drone.sdf

bridge:
    @echo "{{BRIDGE}} Starting ROS-Gazebo bridge..."
    @ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$(pwd)/config/bridge.yaml

chumma2:
    @echo "hi chumma2"

enable-drones:
    @echo "{{DRONE}} Enabling drone controllers..."
    @ign topic -t /drone1/enable -m ignition.msgs.Boolean -p 'data: true'
    @ign topic -t /drone2/enable -m ignition.msgs.Boolean -p 'data: true'
    @ign topic -t /drone3/enable -m ignition.msgs.Boolean -p 'data: true'
    @echo "{{SUCCESS}} All drones enabled!"

setup:
    @echo {{LINK}} " Linking compile_commands.json..."
    @ln -sf build/compile_commands.json .

clean:
    @rm -rf build compile_commands.json
    @echo {{SUCCESS}} " Cleaned."
        


