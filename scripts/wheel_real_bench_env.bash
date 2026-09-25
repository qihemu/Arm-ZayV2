# source本文件准备有人看护的悬空诊断环境；本文件不打开USB、不写电机、不启动节点。
wheel_bench_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$wheel_bench_root/scripts/wheel_env.bash"
export ROS_DOMAIN_ID=55
export ROS_LOCALHOST_ONLY=1
export H55_BENCH_CONFIG="${XDG_CACHE_HOME:-$HOME/.cache}/h55-wheel-control/bench-diagnostic.yaml"
wheel_bench_collection=$(dirname "$wheel_bench_root")
wheel_bench_cxx="/home/G001/.cache/codex-runtimes/codex-primary-runtime/dependencies/native/libheif/libheif/lib"
export LD_LIBRARY_PATH="$wheel_bench_collection/.local/h55-diagnostics/lib:$wheel_bench_cxx:${LD_LIBRARY_PATH:-}"
if ! python3 - "$wheel_bench_root" "$H55_BENCH_CONFIG" <<'PY'
import hashlib
import sys
from pathlib import Path
import yaml

# 只覆盖派生诊断配置中的传输路径，不改根配置的验收标志、限值或电机寄存器。
root, output = map(Path, sys.argv[1:])
sdk = root.parent / '.local/h55-diagnostics/timing-copy/libdm_device.so'
expected = '6723c21c2eec34f5a17baebe4a3dde68f518782d941c416a025e41247838f755'
if hashlib.sha256(sdk.read_bytes()).hexdigest() != expected:
    raise RuntimeError('Diagnostic SDK hash mismatch')
config = yaml.safe_load((root / 'config/robot_wheel_control.yaml').read_text())
wheel = config['robot_wheel_control']
wheel['backend'] = 'direct_usb_sdk'
wheel['operation_mode'] = 'bench'
wheel['transport']['usb']['sdk_library_path'] = str(sdk)
if not wheel['transport']['usb']['serial_number']:
    raise RuntimeError('USB serial_number is empty')
output.parent.mkdir(parents=True, exist_ok=True)
# 同时source多个终端也只会看到完整文件。
import os
temporary = output.with_name(output.name + '.' + str(os.getpid()) + '.tmp')
temporary.write_text(yaml.safe_dump(config, sort_keys=False))
temporary.replace(output)
print('H55 suspended bench only; diagnostic SDK; ROS_DOMAIN_ID=55')
print('Config: ' + str(output))
PY
then
    unset H55_BENCH_CONFIG
    return 1
fi
unset wheel_bench_root wheel_bench_collection wheel_bench_cxx
