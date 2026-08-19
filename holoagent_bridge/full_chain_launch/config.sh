#!/usr/bin/env bash

# All values may be overridden by exporting the same variable before launch.
: "${ISAAC_GPU:=0}"
: "${ISAAC_GUI:=0}"
: "${OVO_GPU:=1}"
: "${HMSG_GPU:=0}"
: "${ROBOT_ID:=13}"
: "${CONTROL_URL:=http://127.0.0.1:8080}"
: "${SIM_READY_TIMEOUT:=180}"
: "${LOCALIZATION_READY_TIMEOUT:=240}"
: "${OVO_READY_TIMEOUT:=300}"
: "${OVO_READY_QUERY:=yellow plastic crate}"

: "${PRIOR_MAP:=${REPO_ROOT}/holoagent_bridge/maps/mid360_filtered_long_20260810_a}"
: "${NAV2_MAP:=${PRIOR_MAP}/grid_map.yaml}"

: "${SEMANTIC_SCENE_ROOT:=${REPO_ROOT}/holoagent_bridge/validation/full_chain_20260819_154652/semantic/live_map_run2}"
: "${HMSG_GRAPH_PATH:=${SEMANTIC_SCENE_ROOT}/graph_20260819160256}"
: "${HMSG_ANCHOR_PATH:=${LAUNCH_DIR}/runtime/semantic_anchors.json}"
: "${SIGLIP_SNAPSHOT:=/home/ubuntu/.cache/modelscope/models/timm--ViT-SO400M-14-SigLIP-384/snapshots/master}"
: "${SAM3_CHECKPOINT:=/home/ubuntu/.cache/modelscope/models/facebook--sam3/snapshots/master/sam3.pt}"

: "${SEMANTIC_PYTHON:=/home/ubuntu/miniconda3/envs/holoagent_semantic_mapping/bin/python}"
: "${AGENT_PYTHON:=/home/ubuntu/miniconda3/bin/python3}"
: "${OVO_CONFIG:=${REPO_ROOT}/HoloAgent/agentic_robot/fsr_vln/configs/ovo_isaac_sam3_local.yaml}"
: "${OVO_DATA_DIR:=${LAUNCH_DIR}/runtime/online_ovo}"
: "${OVO_EXPERIMENT_PREFIX:=full_chain}"
: "${AGENT_OUTPUT_ROOT:=${LAUNCH_DIR}/runtime/agent_task_runs}"

export ISAAC_GPU ISAAC_GUI OVO_GPU HMSG_GPU ROBOT_ID CONTROL_URL
export SIM_READY_TIMEOUT LOCALIZATION_READY_TIMEOUT OVO_READY_TIMEOUT OVO_READY_QUERY
export PRIOR_MAP NAV2_MAP SEMANTIC_SCENE_ROOT HMSG_GRAPH_PATH HMSG_ANCHOR_PATH
export SIGLIP_SNAPSHOT SAM3_CHECKPOINT SEMANTIC_PYTHON AGENT_PYTHON
export OVO_CONFIG OVO_DATA_DIR OVO_EXPERIMENT_PREFIX AGENT_OUTPUT_ROOT
