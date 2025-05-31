#!/bin/bash

# 仮想環境の名前
ENV_NAME="venv"

echo "🔧 仮想環境の作成 ($ENV_NAME)..."
python3 -m venv $ENV_NAME

echo "✅ 仮想環境を作成しました: $ENV_NAME"

echo "📦 ライブラリのインストール（opencv-python, matplotlib）..."
source $ENV_NAME/bin/activate
pip install --upgrade pip
pip install opencv-python matplotlib scipy

echo "✅ インストール完了！"
echo "🧪 仮想環境を有効にするには："
echo "    source $ENV_NAME/bin/activate"
echo " コマンド実行"
echo "🔚 終了するには："
echo "    deactivate"

