colcon build --packages-select voice_assistant_pkg
source install/setup.bash
export SDL_AUDIODRIVER=pulse && export HF_ENDPOINT=https://hf-mirror.com && ros2 run voice_assistant_pkg voice_node