#include "web_server.h"
#include "uart_link.h"
#include "esp_log.h"
#include "esp_http_server.h"

static const char *TAG = "web_server";

// Embedded dashboard page (see EMBED_FILES in CMakeLists.txt).
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");
extern const uint8_t script_js_start[]  asm("_binary_script_js_start");
extern const uint8_t script_js_end[]    asm("_binary_script_js_end");
extern const uint8_t style_css_start[]  asm("_binary_style_css_start");
extern const uint8_t style_css_end[]    asm("_binary_style_css_end");

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t telemetry_handler(httpd_req_t *req)
{
    char buf[512];
    uart_link_get_telemetry(buf, sizeof(buf));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t script_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/javascript");
    httpd_resp_send(req, (const char *)script_js_start, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t style_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/css");
    httpd_resp_send(req, (const char *)style_css_start, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t command_handler(httpd_req_t *req)
{
    char buf[64];
    int len = req->content_len;
    if (len <= 0 || len >= (int)sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad command");
        return ESP_FAIL;
    }
    int n = httpd_req_recv(req, buf, len);
    if (n <= 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
        return ESP_FAIL;
    }
    buf[n] = '\0';
    uart_link_send(buf);
    httpd_resp_sendstr(req, "ok");
    return ESP_OK;
}

void web_server_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }

    httpd_uri_t index_uri = {
        .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
    httpd_register_uri_handler(server, &index_uri);

    httpd_uri_t telemetry_uri = {
        .uri = "/telemetry", .method = HTTP_GET, .handler = telemetry_handler, .user_ctx = NULL };
    httpd_register_uri_handler(server, &telemetry_uri);

    httpd_uri_t script_uri = {
        .uri = "/script.js", .method = HTTP_GET, .handler = script_handler, .user_ctx = NULL };
    httpd_register_uri_handler(server, &script_uri);

    httpd_uri_t style_uri = {
        .uri = "/style.css", .method = HTTP_GET, .handler = style_handler, .user_ctx = NULL };
    httpd_register_uri_handler(server, &style_uri);

    httpd_uri_t command_uri = {
        .uri = "/command", .method = HTTP_POST, .handler = command_handler, .user_ctx = NULL };
    httpd_register_uri_handler(server, &command_uri);

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
}