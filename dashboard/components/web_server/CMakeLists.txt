idf_component_register(SRCS "web_server.c"
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES "index.html" "script.js" "style.css"
                       REQUIRES esp_http_server uart_link log)