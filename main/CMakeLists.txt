idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES bsec2 esp_matter matter_time_sync
    PRIV_REQUIRES esp_timer driver nvs_flash openthread
)
