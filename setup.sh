#!/bin/bash

# --- Rusty-Board Setup Script ---
# This script installs Rust and system dependencies for Linux/macOS.

set -e

echo "🚀 Starting Rusty-Board setup..."

# 1. Detect OS
OS_TYPE="$(uname)"

if [ "$OS_TYPE" == "Linux" ]; then
    echo "📦 Detected Linux. Checking for system dependencies..."
    
    # Check for Debian/Ubuntu based systems
    if command -v apt-get &> /dev/null; then
        echo "🔧 Installing dependencies via apt..."
        sudo apt-get update
        sudo apt-get install -y pkg-config libx11-dev libxi-dev libgl1-mesa-dev libasound2-dev
    else
        echo "⚠️  Non-Debian system detected. Please ensure you have the following installed:"
        echo "   pkg-config, x11, xi, gl, asound development headers."
    fi

elif [ "$OS_TYPE" == "Darwin" ]; then
    echo "🍎 Detected macOS. No additional system dependencies usually required for macroquad."
fi

# 2. Check for Rust
if ! command -v cargo &> /dev/null; then
    echo "🦀 Rust not found. Installing via rustup..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    source "$HOME/.cargo/env"
else
    echo "✅ Rust is already installed."
fi

# 3. Build the project
echo "🛠 Building Rusty-Board..."
cargo build

echo "✅ Setup complete! You can now run the simulation with: cargo run"
