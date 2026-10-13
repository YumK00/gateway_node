# MiddleWare 组件：中间层（业务/协议逻辑），依赖 BSP 提供的硬件能力
#
# SRC_DIRS  / INCLUDE_DIRS：扫描本目录下各子模块的源码与头文件
#     上层直接 #include "xx.h" 即可，不用写相对路径
# REQUIRES：声明本组件用到的其他组件，缺了会在编译或链接阶段报错


#   esp_wifi    → WiFi 驱动
#   nvs_flash   → WiFi 参数存在 NVS 里，初始化前要 nvs_flash_init()
#   esp_event   → WiFi 事件回调机制
#   esp_netif   → TCP/IP 网络接口层
#   mqtt        → MQTT 客户端（连 broker 收发消息）
#   driver      → TWAI 驱动（CAN 控制器）在这里面，头文件是 driver/twai.h
# 注意：freertos / log / esp_common 等属于 IDF 公共依赖，无需显式声明
idf_component_register(
    SRC_DIRS "WIFI" "MQTT" "PROTOCOL" "CAN"
    INCLUDE_DIRS "WIFI" "MQTT" "PROTOCOL" "CAN"
    REQUIRES esp_wifi nvs_flash esp_event esp_netif mqtt driver
)
