#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Captive portal for the SoftAP gallery.
 *
 * When a client (e.g. a Windows PC) joins the open AP it runs an internet
 * connectivity probe. With no upstream internet the OS decides the network is
 * useless and disconnects in a tight loop, so the gallery page never stays
 * reachable. This module makes the AP a captive portal:
 *   - a tiny DNS server answers every query with the AP IP, so all probe
 *     requests land on our HTTP server;
 *   - the HTTP 404 handler redirects any unknown path to /gallery, which the
 *     OS treats as a sign-in page and pops open automatically.
 *
 * Call captive_portal_start() once, after the HTTP server is up and the AP is
 * running. Safe to call again on reconnect; it is idempotent.
 */
esp_err_t captive_portal_start(httpd_handle_t httpd);

/* Stop the DNS server and free its socket/task. */
void captive_portal_stop(void);

#ifdef __cplusplus
}
#endif
