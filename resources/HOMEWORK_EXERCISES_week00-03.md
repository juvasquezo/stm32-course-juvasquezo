sudo cat /proc/driver/nvidia/clients

sudo cat /proc/driver/nvidia/gpus/0000:01:00.0/information

sudo fuser -v /dev/nvidia0 /dev/nvidiactl /dev/nvidia-modeset

watch -n 1 'cat /sys/bus/pci/devices/0000:01:00.0/power/runtime_usage; cat /sys/bus/pci/devices/0000:01:00.0/power/runtime_status'
