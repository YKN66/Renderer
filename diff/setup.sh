#!/bin/bash

# 仮想環境の名前
ENV_NAME="venv"

echo "仮想環境の作成 ($ENV_NAME)..."
python3 -m venv $ENV_NAME
source $ENV_NAME/bin/activate

pip install --upgrade pip
pip install opencv-python matplotlib scipy

echo "仮想環境を有効にするには："
echo "    source $ENV_NAME/bin/activate"
echo " コマンド実行するには"
echo "python3 diff.py"
echo "終了するには："
echo "    deactivate"

