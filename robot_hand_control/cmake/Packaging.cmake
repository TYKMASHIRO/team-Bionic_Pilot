# 打包模块（CPack，一期仅占位）
# 后续如需安装/打包，在此配置 CPack 生成规则。

include(GNUInstallDirs)
include(CPack)

set(CPACK_PACKAGE_NAME "robot_hand_control")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "RM75-6F + LinkerHand O6 control software")
