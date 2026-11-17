#!/bin/bash
# Train attempts until the RL policy reaches >= 90 % success on 40 hard test walls (max 4 attempts).
# Usage: bash auto_train.sh     (from sim/)
set -u
PY=../.venv/bin/python
TARGET=90
CONFIGS=(
  "--lr 1e-4 --log-std -2.0 --seed 0 --steps 3000000"
  "--lr 3e-5 --log-std -2.5 --seed 1 --steps 3000000"
  "--lr 1e-4 --log-std -1.5 --seed 2 --steps 4000000"
  "--lr 5e-5 --log-std -2.0 --seed 3 --steps 5000000"
)
for i in "${!CONFIGS[@]}"; do
  n=$((i + 3))
  echo "=== attempt $n: ${CONFIGS[$i]} ==="
  rm -rf runs
  $PY -W ignore train.py --envs 8 --eval-every 150000 ${CONFIGS[$i]} 2>&1 | grep '\[eval\]'
  $PY -W ignore train.py --eval runs/best/best_model.zip 2>&1 | grep SUCCESS | tee "eval_attempt$n.txt"
  rm -rf "runs_attempt$n"; mv runs "runs_attempt$n"
  rate=$(grep "hard walls" "eval_attempt$n.txt" | grep "RL policy" | sed -E 's/.*SUCCESS ([0-9]+)%.*/\1/')
  echo "attempt $n: RL success on hard test walls = ${rate}%"
  if [ "${rate:-0}" -ge "$TARGET" ]; then
    echo "TARGET REACHED in attempt $n"
    exit 0
  fi
done
echo "TARGET NOT REACHED after ${#CONFIGS[@]} attempts"
