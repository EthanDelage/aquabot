ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/aquabot_gz/worlds"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/aquabot_gz/models"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/aquabot_gz/models/tmp"
ament_prepend_unique_value GZ_SIM_SYSTEM_PLUGIN_PATH "$AMENT_CURRENT_PREFIX/lib"

ament_prepend_unique_value LD_LIBRARY_PATH "$AMENT_CURRENT_PREFIX/lib"
