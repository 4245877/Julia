# Keep runtime resources, excluding editable Blender sources and model archives.
# COPY preserves timestamps and skips unchanged files. The target still runs
# when only a shader or texture changed.
if(NOT IS_DIRECTORY "${JULIA_ASSET_SOURCE}" OR NOT JULIA_ASSET_DESTINATION)
    message(FATAL_ERROR "Runtime assets require a source and destination")
endif()
file(COPY "${JULIA_ASSET_SOURCE}/"
    DESTINATION "${JULIA_ASSET_DESTINATION}"
    REGEX "/models/backups(/|$)" EXCLUDE
    REGEX "\\.blend[0-9]*$" EXCLUDE
)
