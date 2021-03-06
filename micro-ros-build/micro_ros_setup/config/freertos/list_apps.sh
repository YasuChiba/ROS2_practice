function print_available_apps {
  echo "Available apps for FreeRTOS and $PLATFORM:"
  pushd $FW_TARGETDIR/freertos_apps/apps >/dev/null
  for app in $(ls -d */ | cut -f1 -d'/'); do 
    echo "+-- $app"
  done
  popd >/dev/null
}

function check_available_app {
  pushd $FW_TARGETDIR/freertos_apps/apps >/dev/null
    if [ ! -d $1 ]; then
        echo "App $1 for FreeRTOS not available"
        print_available_apps
        exit 1
    fi
  popd >/dev/null
}