#!/bin/bash

# build_plugins_simple.sh

echo "编译Gazebo插件..."

for plugin in bidir_motor_model buoyancy hydrodynamics; do
    echo "处理: $plugin"
    
    cd "$plugin" || continue
    
    # 清理并重新构建
    rm -rf build
    mkdir build
    cd build
    
    cmake .. && make -j$(nproc)
    
    if [ $? -eq 0 ]; then
        echo "✓ $plugin 成功"
    else
        echo "✗ $plugin 失败"
    fi
    
    cd ../..
done

echo "完成!"
