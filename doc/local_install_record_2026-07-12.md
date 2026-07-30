# Local install record: Isaac Sim 5.1 / unitree_sim_env

Date: 2026-07-12

This machine was configured with the native Conda/Pip path, not Docker. The Docker section in the README is an alternative deployment path and was not used.

## Host

- OS workspace: `/home/ubuntu/agents/unitree_sim_isaaclab`
- Conda env: `unitree_sim_env`
- Python: `3.11.15`
- GPUs: 2x NVIDIA GeForce RTX 4090, driver `535.183.01`, 24564 MiB each
- Display session during setup: `DISPLAY=:10.0`, `XDG_SESSION_TYPE=x11`

## Installed locations

- Project: `/home/ubuntu/agents/unitree_sim_isaaclab`
- IsaacLab: `/home/ubuntu/agents/IsaacLab`
- CycloneDDS: `/home/ubuntu/agents/cyclonedds/install`
- unitree_sdk2_python: `/home/ubuntu/agents/unitree_sdk2_python`
- Project assets: `/home/ubuntu/agents/unitree_sim_isaaclab/assets`
- WebRTC certs:
  - `/home/ubuntu/.config/xr_teleoperate/cert.pem`
  - `/home/ubuntu/.config/xr_teleoperate/key.pem`

## Setup command used

Initial command requested:

```bash
bash auto_setup_env.sh 5.1 unitree_sim_env
```

The script's OpenSSL certificate step is interactive, so the successful rerun used blank input:

```bash
yes "" | bash auto_setup_env.sh 5.1 unitree_sim_env
```

The script later stopped at IsaacLab install because the terminal type was `dumb`:

```text
tabs: terminal type 'dumb' cannot reset tabs
```

IsaacLab was continued manually with:

```bash
export TERM=xterm-256color
export CYCLONEDDS_HOME=/home/ubuntu/agents/cyclonedds/install
conda activate unitree_sim_env
cd /home/ubuntu/agents/IsaacLab
./isaaclab.sh --install
```

## Important manual corrections

IsaacLab extra RL dependencies tried to pull newer `torch`/CUDA packages, and `unitree_sdk2_python` dependency resolution tried to upgrade `numpy` to 2.x. These were avoided/restored to keep Isaac Sim 5.1 compatible.

Final pinned/restored compatibility choices:

- `torch==2.7.0+cu128`
- `torchvision==0.22.0+cu128`
- `torchaudio==2.7.0+cu128`
- `numpy==1.26.0`
- `opencv-python==4.11.0.86`
- `cryptography==44.0.0`
- `psutil==5.9.8`
- `typing_extensions==4.12.2`
- `starlette==0.45.3`

## Current key package versions

```text
torch==2.7.0+cu128
torchvision==0.22.0+cu128
torchaudio==2.7.0+cu128
isaacsim==5.1.0.0
isaacsim-core==5.1.0.0
isaaclab==0.54.4
isaaclab_rl==0.5.2
unitree_sdk2py==1.0.1
teleimager==1.5.0
cyclonedds==0.10.2
numpy==1.26.0
opencv-python==4.11.0.86
onnxruntime==1.22.1
rerun-sdk==0.20.1
aiortc==1.14.0
cryptography==44.0.0
starlette==0.45.3
fastapi==0.115.7
psutil==5.9.8
typing_extensions==4.12.2
```

## Known remaining warning

`pip check` reports one metadata conflict:

```text
isaaclab 0.54.4 has requirement starlette==0.49.1, but you have starlette 0.45.3.
```

This was left as-is because `starlette==0.45.3` satisfies the Isaac Sim/FastAPI side. Basic imports passed:

```text
unitree_sdk2py import ok
teleimager import ok
isaaclab import ok
isaaclab_rl import ok
```

## Recommended shell setup

Add CycloneDDS to new terminals:

```bash
echo 'export CYCLONEDDS_HOME=/home/ubuntu/agents/cyclonedds/install' >> ~/.bashrc
source ~/.bashrc
```

Optional explicit WebRTC cert paths:

```bash
echo 'export XR_TELEOP_CERT=$HOME/.config/xr_teleoperate/cert.pem' >> ~/.bashrc
echo 'export XR_TELEOP_KEY=$HOME/.config/xr_teleoperate/key.pem' >> ~/.bashrc
source ~/.bashrc
```

## Quick validation commands

```bash
conda activate unitree_sim_env
cd /home/ubuntu/agents/unitree_sim_isaaclab

python - <<'PY'
import torch
print(torch.__version__)
print(torch.ones(1, device="cuda").item())
PY
```

GUI run:

```bash
python sim_main.py --device cuda --enable_cameras --task Isaac-PickPlace-Cylinder-G129-Dex1-Joint --enable_dex1_dds --robot_type g129
```

Remote/headless run:

```bash
python sim_main.py --device cuda --enable_cameras --task Isaac-PickPlace-Cylinder-G129-Dex1-Joint --enable_dex1_dds --robot_type g129 --no_render --public_ip 127.0.0.1
```
