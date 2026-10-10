import subprocess

commands = [
    ['docker', 'exec', 'graey', 'sh', '-c',
     'ls -l /dev/serial/by-id/; printf "\\nWAIT CHANNELS\\n"; '
     'cat /proc/80/wchan /proc/82/wchan /proc/84/wchan'],
    ['docker', 'exec', 'graey', 'sh', '-c',
     'for n in vn100_node dvl_node; do '
     'echo SOURCE:$n; head -150 /root/robotx_ws/src/robotx_graey_2026/robotx_graey_2026/api/navigation/$n.py; done'],
    ['docker', 'exec', 'graey', 'sh', '-c',
     'ls -l /root/robotx_ws/install/robotx_graey_2026/lib/python3.10/site-packages/; '
     'cat /tmp/launch_params_3unqqu9p'],
]
for cmd in commands:
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    print(r.stdout, r.stderr, flush=True)
