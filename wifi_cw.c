/* =============================================================================
 * wifi_cw.c  –  WiFi (STA/AP), UDP cwdaemon :6789, HTTP control UI :80,
 *               on-chip flash credential storage, Core1 CW queue keyer.
 *
 * Core 0 API : wifi_cw_init(), wifi_cw_task()
 * Core 1 API : wifi_cw_core1_keyer_tick()  (inside g_cw_test_mode==1 path)
 * =============================================================================
 */

#include "wifi_cw.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/flash.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "hardware/watchdog.h"

#include "lwip/udp.h"
#include "lwip/tcp.h"
#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

#include "gpsdo.h"

/* ============================================================
 * Compile-time size check
 * ============================================================ */
_Static_assert(sizeof(wifi_creds_t) == FLASH_SECTOR_SIZE,
               "wifi_creds_t must be exactly one flash sector (4096 bytes)");

/* ============================================================
 * Internal constants
 * ============================================================ */
#define AP_SSID          "QO100TX"
#define AP_CHANNEL       6
#define CLIENT_TIMEOUT_MS 12000u

/* DHCP */
#define DHCP_SERVER_PORT  67u
#define DHCP_CLIENT_PORT  68u
#define DHCP_DISCOVER     1u
#define DHCP_OFFER        2u
#define DHCP_REQUEST      3u
#define DHCP_ACK          5u
#define DHCP_MAGIC        0x63825363ul

/* HTTP response buffer (must fit in lwIP send buffer ~5840 B) */
#define HTTP_BUF_SIZE   3072u
#define HTTP_REQ_SIZE   1024u

/* Power index → dBm table */
static const int8_t k_pwr_dbm[6] = { -18, -12, -6, 0, 6, 13 };
static const char  *k_pwr_label[6] = {
    "-18 dBm", "-12 dBm", "-6 dBm", "0 dBm", "+6 dBm", "+13 dBm"
};

/* ============================================================
 * Morse table  (indexed by char - 0x20, range 0x20..0x5A)
 * ' ' entry is the word-space sentinel (handled separately).
 * ============================================================ */
static const char * const k_morse[59] = {
    /* 0x20 ' '  */ " ",
    /* 0x21 '!'  */ NULL,
    /* 0x22 '"'  */ NULL,
    /* 0x23 '#'  */ NULL,
    /* 0x24 '$'  */ NULL,
    /* 0x25 '%'  */ NULL,
    /* 0x26 '&'  */ NULL,
    /* 0x27 '\'' */ NULL,
    /* 0x28 '('  */ NULL,
    /* 0x29 ')'  */ NULL,
    /* 0x2A '*'  */ NULL,
    /* 0x2B '+'  */ ".-.-.",
    /* 0x2C ','  */ "--..--",
    /* 0x2D '-'  */ "-....-",
    /* 0x2E '.'  */ ".-.-.-",
    /* 0x2F '/'  */ "-..-.",
    /* 0x30 '0'  */ "-----",
    /* 0x31 '1'  */ ".----",
    /* 0x32 '2'  */ "..---",
    /* 0x33 '3'  */ "...--",
    /* 0x34 '4'  */ "....-",
    /* 0x35 '5'  */ ".....",
    /* 0x36 '6'  */ "-....",
    /* 0x37 '7'  */ "--...",
    /* 0x38 '8'  */ "---..",
    /* 0x39 '9'  */ "----.",
    /* 0x3A ':'  */ NULL,
    /* 0x3B ';'  */ NULL,
    /* 0x3C '<'  */ NULL,
    /* 0x3D '='  */ "-...-",
    /* 0x3E '>'  */ NULL,
    /* 0x3F '?'  */ "..--..",
    /* 0x40 '@'  */ ".--.-.",
    /* 0x41 'A'  */ ".-",
    /* 0x42 'B'  */ "-...",
    /* 0x43 'C'  */ "-.-.",
    /* 0x44 'D'  */ "-..",
    /* 0x45 'E'  */ ".",
    /* 0x46 'F'  */ "..-.",
    /* 0x47 'G'  */ "--.",
    /* 0x48 'H'  */ "....",
    /* 0x49 'I'  */ "..",
    /* 0x4A 'J'  */ ".---",
    /* 0x4B 'K'  */ "-.-",
    /* 0x4C 'L'  */ ".-..",
    /* 0x4D 'M'  */ "--",
    /* 0x4E 'N'  */ "-.",
    /* 0x4F 'O'  */ "---",
    /* 0x50 'P'  */ ".--.",
    /* 0x51 'Q'  */ "--.-",
    /* 0x52 'R'  */ ".-.",
    /* 0x53 'S'  */ "...",
    /* 0x54 'T'  */ "-",
    /* 0x55 'U'  */ "..-",
    /* 0x56 'V'  */ "...-",
    /* 0x57 'W'  */ ".--",
    /* 0x58 'X'  */ "-..-",
    /* 0x59 'Y'  */ "-.--",
    /* 0x5A 'Z'  */ "--.."
};

static inline const char *morse_for(char c) {
    uint8_t u = (uint8_t)c;
    if (u < 0x20u || u > 0x5Au) return NULL;
    return k_morse[u - 0x20u];
}

/* ============================================================
 * State shared between Core0 callbacks and Core1 keyer
 * (volatile; Core0 IRQ writes, Core1 reads and vice-versa)
 * ============================================================ */
static queue_t          *s_cw_queue        = NULL;
static volatile uint8_t *s_ptt             = NULL;
static volatile uint8_t *s_tx_mode         = NULL;
static volatile uint8_t *s_cw_test_mode    = NULL;
static volatile int8_t  *s_tx_power_dbm    = NULL;
static volatile double  *s_target_freq     = NULL;
static volatile uint8_t *s_tx_enabled      = NULL;
static volatile float   *s_ppm             = NULL;

static volatile uint8_t  s_wifi_wpm        = 20u; /* WPM for queue keyer   */
static volatile bool     s_abort_pending   = false;
static char              s_ip_str[32]      = "";
static bool              s_wifi_ok         = false;

/* Deferred flash-save (set from HTTP callback, actioned by wifi_cw_task) */
static volatile bool     s_save_pending    = false;
static wifi_creds_t      s_pending_creds;  /* written before s_save_pending */

/* ============================================================
 * Flash helpers
 * ============================================================ */
static void load_wifi_creds(wifi_creds_t *out) {
    const wifi_creds_t *stored =
        (const wifi_creds_t *)(XIP_BASE + WIFI_CREDS_FLASH_OFFSET);
    if (stored->magic == WIFI_CREDS_MAGIC) {
        *out = *stored;
    } else {
        memset(out, 0, sizeof(*out));
    }
}

/* Passed as the callback to flash_safe_execute (interrupts already off,
 * XIP suspended, Core1 paused by flash_safe_execute). */
static void do_flash_save(void *param) {
    (void)param;
    flash_range_erase(WIFI_CREDS_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(WIFI_CREDS_FLASH_OFFSET,
                        (const uint8_t *)&s_pending_creds,
                        sizeof(s_pending_creds));
}

static void save_wifi_creds(const char *ssid, const char *password) {
    memset(&s_pending_creds, 0, sizeof(s_pending_creds));
    s_pending_creds.magic = WIFI_CREDS_MAGIC;
    strncpy(s_pending_creds.ssid,     ssid,     sizeof(s_pending_creds.ssid)     - 1u);
    strncpy(s_pending_creds.password, password, sizeof(s_pending_creds.password) - 1u);
    /* flash_safe_execute pauses Core1 and disables IRQs automatically. */
    flash_safe_execute(do_flash_save, NULL, 2000u);
}

/* ============================================================
 * URL helpers
 * ============================================================ */
static void url_decode_inplace(char *s) {
    char *r = s, *w = s;
    while (*r) {
        if (*r == '+') {
            *w++ = ' '; r++;
        } else if (r[0] == '%' && isxdigit((unsigned char)r[1]) &&
                                  isxdigit((unsigned char)r[2])) {
            char hex[3] = { r[1], r[2], '\0' };
            *w++ = (char)(int)strtol(hex, NULL, 16);
            r += 3;
        } else {
            *w++ = *r++;
        }
    }
    *w = '\0';
}

/* Extract the value of key= from an application/x-www-form-urlencoded body.
 * Returns 1 on success, 0 if key not found. */
static int form_val(const char *body, const char *key,
                    char *out, size_t out_sz) {
    size_t klen = strlen(key);
    const char *p = body;
    while (p && *p) {
        if (strncmp(p, key, klen) == 0 && p[klen] == '=') {
            p += klen + 1;
            size_t n = 0;
            while (*p && *p != '&' && n < out_sz - 1u)
                out[n++] = *p++;
            out[n] = '\0';
            url_decode_inplace(out);
            return 1;
        }
        p = strchr(p, '&');
        if (p) p++;
    }
    out[0] = '\0';
    return 0;
}

static uint8_t dbm_to_idx(int8_t dbm) {
    for (int i = 0; i < 5; i++)
        if (dbm <= k_pwr_dbm[i]) return (uint8_t)i;
    return 5u;
}

/* ============================================================
 * HTTP page builders
 * All write into s_http_resp; return byte count (excl. NUL).
 * ============================================================ */
static char s_http_resp[HTTP_BUF_SIZE];

static int build_main_page(void) {
    /* Snapshot current values (safe: all volatile scalars, Core0 context) */
    double freq_hz  = s_target_freq ? (double)*s_target_freq : 0.0;
    uint32_t fkhz   = (uint32_t)(freq_hz / 1000.0 + 0.5);
    uint8_t  wpm    = s_wifi_wpm;
    uint8_t  pidx   = s_tx_power_dbm ? dbm_to_idx(*s_tx_power_dbm) : 5u;
    uint8_t  txmode = s_tx_mode      ? *s_tx_mode    : 0u;
    uint8_t  txen   = s_tx_enabled   ? *s_tx_enabled : 1u;
    float    ppm    = s_ppm          ? *s_ppm        : 0.0f;

    /* GPSDO status */
    char gpsdo_buf[128];
    gpsdo_format_status(gpsdo_buf, sizeof(gpsdo_buf));

    int n = 0;
    /* HTTP header */
    n += snprintf(s_http_resp + n, HTTP_BUF_SIZE - (size_t)n,
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Connection: close\r\n\r\n"
        "<!DOCTYPE html><html><head>"
        "<meta charset=utf-8>"
        "<meta name=viewport content=\"width=device-width,initial-scale=1\">"
        "<meta http-equiv=refresh content=10>"
        "<title>QO-100 TX</title>"
        "<style>"
        "body{font-family:sans-serif;max-width:420px;margin:16px auto;padding:0 10px}"
        "h2{margin-bottom:4px}h3{margin:10px 0 4px}"
        "label{display:block;margin:5px 0}"
        "input,select{width:100%%;box-sizing:border-box;padding:4px}"
        "button{padding:6px 18px;margin-top:8px;cursor:pointer}"
        ".status{background:#f4f4f4;border:1px solid #ccc;padding:8px;"
                "border-radius:4px;font-family:monospace;font-size:0.85em;"
                "white-space:pre-wrap;margin-bottom:10px}"
        ".row{display:flex;gap:8px}"
        ".row button{flex:1}"
        "</style></head><body>"
        "<h2>QO-100 TX &mdash; %s</h2>"
        "<div class=status>"
        "Freq : %.3f MHz\n"
        "Mode : %s\n"
        "TX   : %s  Power: %s  PPM: %+.2f\n"
        "%s"
        "</div>",
        s_ip_str,
        freq_hz / 1e6,
        txmode ? "CW" : "USB/SSB",
        txen   ? "ON" : "OFF",
        k_pwr_label[pidx],
        (double)ppm,
        gpsdo_buf
    );

    /* Settings form */
    n += snprintf(s_http_resp + n, HTTP_BUF_SIZE - (size_t)n,
        "<h3>Settings</h3>"
        "<form method=POST action=/set>"
        "<label>Freq (kHz):<input name=freq type=number value=%lu"
              " min=2400000 max=2500000></label>"
        "<label>WPM:<input name=wpm type=number value=%u min=5 max=60></label>"
        "<label>Power:<select name=pwr>",
        (unsigned long)fkhz, (unsigned)wpm);

    for (int i = 0; i < 6; i++) {
        n += snprintf(s_http_resp + n, HTTP_BUF_SIZE - (size_t)n,
            "<option value=%d%s>%s</option>",
            i, (i == (int)pidx) ? " selected" : "", k_pwr_label[i]);
    }

    n += snprintf(s_http_resp + n, HTTP_BUF_SIZE - (size_t)n,
        "</select></label>"
        "<label>PPM:<input name=ppm type=number step=0.01 value=%.2f></label>"
        "<div class=row>"
        "<button name=mode value=usb%s type=submit>USB/SSB</button>"
        "<button name=mode value=cw%s  type=submit>CW Mode</button>"
        "</div>"
        "<button type=submit>Apply</button>"
        "</form>",
        (double)ppm,
        txmode ? "" : " style=font-weight:bold",
        txmode ? " style=font-weight:bold" : "");

    /* CW send form */
    n += snprintf(s_http_resp + n, HTTP_BUF_SIZE - (size_t)n,
        "<h3>Send CW</h3>"
        "<form method=POST action=/send>"
        "<label>Text:<input name=text value=\"CQ CQ DE TEST\"></label>"
        "<button type=submit>&#9654; Send CW</button>"
        "</form>");

    /* Footer */
    n += snprintf(s_http_resp + n, HTTP_BUF_SIZE - (size_t)n,
        "<p style='margin-top:14px'><a href=/wifi>&#9881; WiFi Settings</a></p>"
        "</body></html>");

    return n;
}

static const char k_wifi_page[] =
    "HTTP/1.0 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
    "Connection: close\r\n\r\n"
    "<!DOCTYPE html><html><head><meta charset=utf-8>"
    "<meta name=viewport content=\"width=device-width,initial-scale=1\">"
    "<title>WiFi Setup</title>"
    "<style>body{font-family:sans-serif;max-width:380px;margin:16px auto;padding:0 10px}"
    "label{display:block;margin:6px 0}"
    "input{width:100%;box-sizing:border-box;padding:4px}"
    "button{padding:6px 18px;margin-top:8px}</style></head>"
    "<body><h2>WiFi Setup</h2>"
    "<p>Leave password empty for open network.</p>"
    "<form method=POST action=/wifi>"
    "<label>SSID:<input name=ssid required></label>"
    "<label>Password:<input name=password type=password></label>"
    "<button type=submit>Save &amp; Reboot</button></form>"
    "<p><a href=/>&#8592; Back</a></p></body></html>";

static const char k_saving_page[] =
    "HTTP/1.0 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
    "Connection: close\r\n\r\n"
    "<!DOCTYPE html><html><body>"
    "<h2>Saving&#8230;</h2>"
    "<p>Credentials saved. Rebooting into client mode&hellip;</p>"
    "</body></html>";

static const char k_redirect_root[] =
    "HTTP/1.0 303 See Other\r\nLocation: /\r\nConnection: close\r\n\r\n";

static const char k_redirect_wifi[] =
    "HTTP/1.0 303 See Other\r\nLocation: /wifi\r\nConnection: close\r\n\r\n";

static const char k_not_found[] =
    "HTTP/1.0 404 Not Found\r\nContent-Type: text/plain\r\n"
    "Connection: close\r\n\r\n404 Not Found\r\n";

/* ============================================================
 * HTTP server  (lwIP raw TCP, one connection at a time)
 * ============================================================ */
typedef struct {
    struct tcp_pcb *pcb;
    char  buf[HTTP_REQ_SIZE];
    int   len;
} http_conn_t;

static http_conn_t s_hconn;

static void http_close(struct tcp_pcb *pcb) {
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, NULL);
    tcp_err(pcb, NULL);
    tcp_close(pcb);
}

static void http_send_const(struct tcp_pcb *pcb,
                             const char *data, size_t len) {
    tcp_write(pcb, data, (u16_t)len, TCP_WRITE_FLAG_COPY);
    tcp_output(pcb);
}

/* Process a complete HTTP request stored in conn->buf. */
static void http_process(struct tcp_pcb *pcb, http_conn_t *conn) {
    char method[8] = {0}, path[64] = {0};
    sscanf(conn->buf, "%7s %63s", method, path);

    /* Strip query string */
    char *qs = strchr(path, '?');
    if (qs) *qs = '\0';

    /* Locate body (after blank line) */
    char *body = strstr(conn->buf, "\r\n\r\n");
    if (body) body += 4;

    if (strcmp(method, "GET") == 0) {
        if (strcmp(path, "/") == 0) {
            int n = build_main_page();
            tcp_write(pcb, s_http_resp, (u16_t)n, TCP_WRITE_FLAG_COPY);
            tcp_output(pcb);
        } else if (strcmp(path, "/wifi") == 0) {
            http_send_const(pcb, k_wifi_page, sizeof(k_wifi_page) - 1u);
        } else {
            http_send_const(pcb, k_not_found, sizeof(k_not_found) - 1u);
        }

    } else if (strcmp(method, "POST") == 0 && body) {
        char val[128];

        if (strcmp(path, "/send") == 0) {
            if (form_val(body, "text", val, sizeof(val))) {
                for (char *p = val; *p; p++) {
                    char c = (char)toupper((unsigned char)*p);
                    if (c == ' ' || (c >= '!' && c <= 'Z'))
                        queue_try_add(s_cw_queue, &c);
                }
            }
            http_send_const(pcb, k_redirect_root, sizeof(k_redirect_root) - 1u);

        } else if (strcmp(path, "/set") == 0) {
            if (form_val(body, "freq", val, sizeof(val))) {
                long f = atol(val);
                if (f >= 2400000L && f <= 2500000L && s_target_freq)
                    *s_target_freq = (double)f * 1000.0;
            }
            if (form_val(body, "wpm", val, sizeof(val))) {
                int w = atoi(val);
                if (w >= 5 && w <= 60) s_wifi_wpm = (uint8_t)w;
            }
            if (form_val(body, "pwr", val, sizeof(val))) {
                int pi = atoi(val);
                if (pi >= 0 && pi <= 5 && s_tx_power_dbm)
                    *s_tx_power_dbm = k_pwr_dbm[pi];
            }
            if (form_val(body, "ppm", val, sizeof(val))) {
                double p = strtod(val, NULL);
                if (p >= -100.0 && p <= 100.0 && s_ppm)
                    *s_ppm = (float)p;
            }
            if (form_val(body, "mode", val, sizeof(val))) {
                if (s_tx_mode) {
                    if (strcmp(val, "cw") == 0)  *s_tx_mode = 1u;
                    else                          *s_tx_mode = 0u;
                }
            }
            http_send_const(pcb, k_redirect_root, sizeof(k_redirect_root) - 1u);

        } else if (strcmp(path, "/wifi") == 0) {
            char ssid[64] = {0}, pass[64] = {0};
            form_val(body, "ssid",     ssid, sizeof(ssid));
            form_val(body, "password", pass, sizeof(pass));
            if (ssid[0]) {
                /* Copy into pending struct; actual flash write happens in
                 * wifi_cw_task() on Core0 main loop (not from IRQ). */
                memset(&s_pending_creds, 0, sizeof(s_pending_creds));
                s_pending_creds.magic = WIFI_CREDS_MAGIC;
                strncpy(s_pending_creds.ssid,     ssid, 63u);
                strncpy(s_pending_creds.password, pass, 63u);
                __compiler_memory_barrier();
                s_save_pending = true;
                http_send_const(pcb, k_saving_page, sizeof(k_saving_page) - 1u);
            } else {
                http_send_const(pcb, k_redirect_wifi,
                                sizeof(k_redirect_wifi) - 1u);
            }
        } else {
            http_send_const(pcb, k_not_found, sizeof(k_not_found) - 1u);
        }
    }

    http_close(pcb);
}

static void http_err_cb(void *arg, err_t err) {
    (void)arg; (void)err;
    s_hconn.pcb = NULL;
}

static err_t http_recv_cb(void *arg, struct tcp_pcb *pcb,
                           struct pbuf *p, err_t err) {
    http_conn_t *conn = (http_conn_t *)arg;

    if (!p) { http_close(pcb); return ERR_OK; }
    if (err != ERR_OK) { pbuf_free(p); return err; }

    /* Accumulate into buffer */
    struct pbuf *cur = p;
    while (cur && conn->len < (int)sizeof(conn->buf) - 1) {
        int copy = (int)cur->len;
        if (conn->len + copy > (int)sizeof(conn->buf) - 1)
            copy = (int)sizeof(conn->buf) - 1 - conn->len;
        memcpy(conn->buf + conn->len, cur->payload, (size_t)copy);
        conn->len += copy;
        cur = cur->next;
    }
    conn->buf[conn->len] = '\0';
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);

    /* Check completeness */
    char *hend = strstr(conn->buf, "\r\n\r\n");
    if (!hend) return ERR_OK;

    bool is_post = (strncmp(conn->buf, "POST", 4) == 0);
    if (is_post) {
        const char *cl = strstr(conn->buf, "Content-Length:");
        if (!cl) cl = strstr(conn->buf, "content-length:");
        int clen = cl ? atoi(cl + 15) : 0;
        int have = conn->len - (int)(hend + 4 - conn->buf);
        if (have < clen) return ERR_OK;
    }

    http_process(pcb, conn);
    return ERR_OK;
}

static err_t http_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err) {
    (void)arg;
    if (err != ERR_OK || !newpcb) return ERR_VAL;
    tcp_setprio(newpcb, TCP_PRIO_MIN);
    memset(&s_hconn, 0, sizeof(s_hconn));
    s_hconn.pcb = newpcb;
    tcp_arg(newpcb, &s_hconn);
    tcp_recv(newpcb, http_recv_cb);
    tcp_err(newpcb, http_err_cb);
    return ERR_OK;
}

static void http_server_init(void) {
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) return;
    if (tcp_bind(pcb, IP4_ADDR_ANY, WIFI_HTTP_PORT) != ERR_OK) {
        tcp_close(pcb); return;
    }
    struct tcp_pcb *lpcb = tcp_listen(pcb);
    if (!lpcb) { tcp_close(pcb); return; }
    tcp_accept(lpcb, http_accept_cb);
}

/* ============================================================
 * Minimal DHCP server for AP mode (one client, fixed lease)
 * Handles DHCPDISCOVER and DHCPREQUEST; always offers .2.
 * ============================================================ */
typedef struct __attribute__((packed)) {
    uint8_t  op, htype, hlen, hops;
    uint32_t xid;
    uint16_t secs, flags;
    uint8_t  ciaddr[4], yiaddr[4], siaddr[4], giaddr[4];
    uint8_t  chaddr[16];
    uint8_t  sname[64];
    uint8_t  file[128];
    uint32_t magic;
    uint8_t  options[312];
} dhcp_msg_t;

static struct udp_pcb  *s_dhcp_pcb  = NULL;
static ip4_addr_t       s_ap_ip;
static ip4_addr_t       s_ap_mask;

static void dhcp_opt(uint8_t **p, uint8_t code, uint8_t len,
                     const void *data) {
    *(*p)++ = code;
    *(*p)++ = len;
    memcpy(*p, data, len);
    *p += len;
}

static void dhcp_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                          const ip_addr_t *addr, u16_t port) {
    (void)arg; (void)addr; (void)port;

    dhcp_msg_t in = {0};
    u16_t copy = p->tot_len < (u16_t)sizeof(in) ? p->tot_len
                                                 : (u16_t)sizeof(in);
    pbuf_copy_partial(p, &in, copy, 0);
    pbuf_free(p);

    if (lwip_ntohl(in.magic) != DHCP_MAGIC) return;

    /* Parse message type option */
    uint8_t mtype = 0;
    const uint8_t *o = in.options, *oend = in.options + sizeof(in.options);
    while (o < oend && *o != 0xFFu) {
        if (*o == 0u) { o++; continue; }
        if (*o == 53u) { mtype = o[2]; break; }
        o += 2u + o[1];
    }
    if (mtype != DHCP_DISCOVER && mtype != DHCP_REQUEST) return;

    /* Build reply */
    dhcp_msg_t out = {0};
    out.op    = 2u;   /* BOOTREPLY */
    out.htype = 1u;   /* Ethernet  */
    out.hlen  = 6u;
    out.xid   = in.xid;
    out.flags = in.flags;

    /* Offered IP: ap_subnet.2 */
    uint32_t offer_ip = (lwip_ntohl(s_ap_ip.addr) & 0xFFFFFF00u) | 0x02u;
    offer_ip = lwip_htonl(offer_ip);
    memcpy(out.yiaddr, &offer_ip, 4u);
    memcpy(out.siaddr, &s_ap_ip.addr, 4u);
    memcpy(out.chaddr, in.chaddr, 16u);
    out.magic = lwip_htonl(DHCP_MAGIC);

    uint8_t *op = out.options;
    uint8_t  mt   = (mtype == DHCP_DISCOVER) ? DHCP_OFFER : DHCP_ACK;
    uint32_t lease = lwip_htonl(24u * 3600u);
    dhcp_opt(&op, 53u, 1u, &mt);
    dhcp_opt(&op, 54u, 4u, &s_ap_ip.addr);      /* server id   */
    dhcp_opt(&op, 51u, 4u, &lease);              /* lease time  */
    dhcp_opt(&op, 1u,  4u, &s_ap_mask.addr);    /* subnet mask */
    dhcp_opt(&op, 3u,  4u, &s_ap_ip.addr);      /* gateway     */
    dhcp_opt(&op, 6u,  4u, &s_ap_ip.addr);      /* DNS         */
    *op++ = 0xFFu;                               /* end option  */

    struct pbuf *rp = pbuf_alloc(PBUF_TRANSPORT,
                                 (u16_t)sizeof(dhcp_msg_t), PBUF_RAM);
    if (!rp) return;
    memcpy(rp->payload, &out, sizeof(dhcp_msg_t));

    /* Send as broadcast */
    ip4_addr_t bcast;
    IP4_ADDR(&bcast, 255, 255, 255, 255);
    udp_sendto_if(s_dhcp_pcb, rp,
                  (const ip_addr_t *)&bcast, DHCP_CLIENT_PORT,
                  netif_default);
    pbuf_free(rp);
}

static void dhcp_server_start(ip4_addr_t *ip, ip4_addr_t *mask) {
    s_ap_ip   = *ip;
    s_ap_mask = *mask;
    s_dhcp_pcb = udp_new();
    if (!s_dhcp_pcb) return;
    udp_bind(s_dhcp_pcb, IP4_ADDR_ANY, DHCP_SERVER_PORT);
    udp_recv(s_dhcp_pcb, dhcp_recv_cb, NULL);
}

/* ============================================================
 * UDP cwdaemon server  (port 6789)
 * ============================================================ */
static void udp_cw_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                             const ip_addr_t *addr, u16_t port) {
    (void)arg; (void)pcb; (void)addr; (void)port;
    if (!p || !s_cw_queue) { if (p) pbuf_free(p); return; }

    char buf[256];
    u16_t len = p->tot_len < (u16_t)(sizeof(buf) - 1u)
                    ? p->tot_len : (u16_t)(sizeof(buf) - 1u);
    pbuf_copy_partial(p, buf, len, 0u);
    buf[len] = '\0';
    pbuf_free(p);

    int i = 0;
    while (i < (int)len) {
        unsigned char ch = (unsigned char)buf[i];
        if (ch == 0x1Bu) {                         /* ESC prefix */
            i++;
            if (i >= (int)len) break;
            char cmd = buf[i++];
            if (cmd == '4') {                      /* \x1b4 – abort */
                s_abort_pending = true;
            } else if (cmd == '2') {               /* \x1b2<digits> – WPM */
                int wpm = 0;
                while (i < (int)len &&
                       isdigit((unsigned char)buf[i]))
                    wpm = wpm * 10 + (buf[i++] - '0');
                if (wpm >= 5 && wpm <= 60)
                    s_wifi_wpm = (uint8_t)wpm;
            }
            /* other ESC commands ignored */
        } else {
            char c = (char)toupper(ch);
            if (c == ' ' || (c >= '!' && c <= 'Z'))
                queue_try_add(s_cw_queue, &c);
            i++;
        }
    }
}

static void udp_cw_server_init(void) {
    struct udp_pcb *pcb = udp_new();
    if (!pcb) return;
    udp_bind(pcb, IP4_ADDR_ANY, WIFI_CW_UDP_PORT);
    udp_recv(pcb, udp_cw_recv_cb, NULL);
}

/* ============================================================
 * Public API
 * ============================================================ */

void wifi_cw_init(queue_t          *cw_queue,
                  volatile uint8_t *p_soft_ptt_key,
                  volatile uint8_t *p_tx_mode,
                  volatile uint8_t *p_cw_test_mode,
                  volatile int8_t  *p_tx_power_dbm,
                  volatile double  *p_target_freq,
                  volatile uint8_t *p_tx_enabled,
                  volatile float   *p_ppm) {
    s_cw_queue      = cw_queue;
    s_ptt           = p_soft_ptt_key;
    s_tx_mode       = p_tx_mode;
    s_cw_test_mode  = p_cw_test_mode;
    s_tx_power_dbm  = p_tx_power_dbm;
    s_target_freq   = p_target_freq;
    s_tx_enabled    = p_tx_enabled;
    s_ppm           = p_ppm;

    if (cyw43_arch_init() != 0) {
        printf("[WiFi] cyw43_arch_init failed\n");
        return;
    }

    wifi_creds_t creds;
    load_wifi_creds(&creds);

    bool connected = false;

    if (creds.magic == WIFI_CREDS_MAGIC && creds.ssid[0] != '\0') {
        printf("[WiFi] Trying STA: %s\n", creds.ssid);
        cyw43_arch_enable_sta_mode();

        uint32_t auth = creds.password[0]
                        ? CYW43_AUTH_WPA2_MIXED_PSK
                        : CYW43_AUTH_OPEN;
        const char *pass = creds.password[0] ? creds.password : NULL;

        if (cyw43_arch_wifi_connect_timeout_ms(
                creds.ssid, pass, auth, CLIENT_TIMEOUT_MS) == 0) {
            /* Connected – read assigned IP from default netif */
            const ip4_addr_t *ip = netif_ip4_addr(netif_default);
            snprintf(s_ip_str, sizeof(s_ip_str), "%s", ip4addr_ntoa(ip));
            connected = true;
            printf("[WiFi] STA connected: %s\n", s_ip_str);
        } else {
            printf("[WiFi] STA failed, starting AP\n");
            cyw43_arch_disable_sta_mode();
        }
    }

    if (!connected) {
        /* AP mode – open, SSID QO100TX */
        cyw43_arch_enable_ap_mode(AP_SSID, NULL, CYW43_AUTH_OPEN);

        /* Configure netif for AP: 192.168.4.1/24 */
        ip4_addr_t ip, mask, gw;
        IP4_ADDR(&ip,   192, 168, 4, 1);
        IP4_ADDR(&mask, 255, 255, 255, 0);
        IP4_ADDR(&gw,   192, 168, 4, 1);
        netif_set_addr(netif_default, &ip, &mask, &gw);

        snprintf(s_ip_str, sizeof(s_ip_str), "AP:192.168.4.1");
        printf("[WiFi] AP mode: %s (open, SSID=%s)\n", s_ip_str, AP_SSID);

        /* Start DHCP server so connecting clients get an address */
        dhcp_server_start(&ip, &mask);
    }

    udp_cw_server_init();
    http_server_init();
    s_wifi_ok = true;
    printf("[WiFi] UDP cwdaemon :%u  HTTP :%u  ready\n",
           WIFI_CW_UDP_PORT, WIFI_HTTP_PORT);
}

void wifi_cw_task(void) {
    if (!s_wifi_ok) return;

    /* Deferred flash save + reboot (must NOT run from IRQ context) */
    if (s_save_pending) {
        s_save_pending = false;
        printf("[WiFi] Saving credentials to flash…\n");
        /* do_flash_save() writes s_pending_creds; pauses Core1 automatically */
        flash_safe_execute(do_flash_save, NULL, 2000u);
        printf("[WiFi] Rebooting…\n");
        watchdog_reboot(0u, 0u, 200u);   /* reboot in 200 ms */
        for (;;) tight_loop_contents();  /* wait for watchdog */
    }

    /* Handle any pending abort flag raised by UDP ESC handler */
    if (s_abort_pending) {
        s_abort_pending = false;
        if (s_ptt) *s_ptt = 0u;
        /* Drain the CW queue */
        char c;
        while (s_cw_queue && queue_try_remove(s_cw_queue, &c)) {}
    }
}

const char *wifi_cw_ip_str(void) {
    return s_ip_str;
}

/* ============================================================
 * Core1 CW queue keyer state machine
 * Called every ~5 ms from the g_cw_test_mode==1 idle path.
 * Non-blocking: uses absolute timestamps for element timing.
 * ============================================================ */
typedef enum {
    KS_IDLE = 0,
    KS_WAIT_ARMED,   /* tx_mode=1 set, waiting for cw_test_mode==1    */
    KS_ELEM_ON,      /* carrier ON, timing element duration            */
    KS_ELEM_OFF,     /* carrier OFF, inter-element gap (1 unit)        */
    KS_CHAR_GAP,     /* between chars (3 units); also entry to keying  */
    KS_WORD_GAP,     /* extra 4 units after a ' ' (7 total)            */
} keyer_state_t;

static struct {
    keyer_state_t  state;
    char           text[WIFI_CW_QUEUE_DEPTH + 1];
    int            text_len;
    int            text_pos;
    const char    *morse;     /* pointer into k_morse[] entry          */
    int            morse_pos; /* index of current element              */
    uint64_t       deadline;  /* time_us_64() when current phase ends  */
    uint8_t        wpm;
} s_k;

void wifi_cw_core1_keyer_tick(void) {
    /* Abort request (written by Core0, read here on Core1) */
    if (s_abort_pending) {
        s_abort_pending = false;
        if (s_ptt) *s_ptt = 0u;
        __compiler_memory_barrier();
        s_k.state    = KS_IDLE;
        s_k.text_len = 0;
        s_k.text_pos = 0;
        return;
    }

    switch (s_k.state) {

    /* ---- IDLE: drain queue into local text buffer ---- */
    case KS_IDLE: {
        s_k.text_len = 0;
        s_k.text_pos = 0;
        char c;
        while (s_k.text_len < (int)(sizeof(s_k.text) - 1u) &&
               s_cw_queue && queue_try_remove(s_cw_queue, &c)) {
            s_k.text[s_k.text_len++] = (char)toupper((unsigned char)c);
        }
        if (s_k.text_len == 0) return;   /* nothing to send */
        s_k.text[s_k.text_len] = '\0';

        /* Request CW mode; carrier_poll() on Core0 will set cw_test_mode */
        if (s_tx_mode) *s_tx_mode = 1u;
        __compiler_memory_barrier();
        s_k.wpm      = s_wifi_wpm;
        s_k.state    = KS_WAIT_ARMED;
        s_k.deadline = time_us_64() + 200000u; /* 200 ms arm timeout */
        return;
    }

    /* ---- WAIT_ARMED: poll until carrier_poll has idled Core1 ---- */
    case KS_WAIT_ARMED:
        if (s_cw_test_mode && *s_cw_test_mode) {
            /* Radio armed; start first char immediately */
            s_k.state    = KS_CHAR_GAP;
            s_k.deadline = time_us_64(); /* expire now */
        } else if (time_us_64() > s_k.deadline) {
            /* Timed out – abort */
            s_k.text_len = 0;
            s_k.state    = KS_IDLE;
        }
        return;

    default:
        break;
    }

    /* ---- Timing states ---- */
    uint64_t now = time_us_64();
    if (now < s_k.deadline) return;   /* still timing */

    uint32_t dit_us = 1200000u / (uint32_t)s_k.wpm;

    switch (s_k.state) {

    case KS_ELEM_ON:
        /* Element done → carrier off, start post-element gap */
        if (s_ptt) { *s_ptt = 0u; __compiler_memory_barrier(); }
        s_k.morse_pos++;
        if (s_k.morse && s_k.morse[s_k.morse_pos] != '\0') {
            /* More elements in this character */
            s_k.state    = KS_ELEM_OFF;
            s_k.deadline = now + dit_us;
        } else {
            /* End of character – 3-unit inter-char gap */
            s_k.state    = KS_CHAR_GAP;
            s_k.deadline = now + 3u * dit_us;
        }
        break;

    case KS_ELEM_OFF:
        /* Gap done → start next element */
        {
            char sym = s_k.morse[s_k.morse_pos];
            uint32_t dur = (sym == '-') ? 3u * dit_us : dit_us;
            if (s_ptt) { *s_ptt = 1u; __compiler_memory_barrier(); }
            s_k.state    = KS_ELEM_ON;
            s_k.deadline = now + dur;
        }
        break;

    case KS_CHAR_GAP:
        /* Advance to next character */
        if (s_k.text_pos >= s_k.text_len) {
            /* All text sent */
            if (s_ptt) *s_ptt = 0u;
            s_k.text_len = 0;
            s_k.text_pos = 0;
            s_k.state    = KS_IDLE;
            break;
        }
        {
            char ch = s_k.text[s_k.text_pos++];
            if (ch == ' ') {
                /* Word gap: 7 units total; 3 already elapsed, need 4 more */
                s_k.state    = KS_WORD_GAP;
                s_k.deadline = now + 4u * dit_us;
                break;
            }
            const char *m = morse_for(ch);
            if (!m || m[0] == ' ' || m[0] == '\0') {
                /* Unknown char: treat as inter-char gap (deadline = now) */
                s_k.deadline = now;
                break;
            }
            s_k.morse     = m;
            s_k.morse_pos = 0;
            char sym = m[0];
            uint32_t dur = (sym == '-') ? 3u * dit_us : dit_us;
            if (s_ptt) { *s_ptt = 1u; __compiler_memory_barrier(); }
            s_k.state    = KS_ELEM_ON;
            s_k.deadline = now + dur;
        }
        break;

    case KS_WORD_GAP:
        /* Extra gap done; loop back to char processing (text_pos already
         * advanced past the space) */
        s_k.state    = KS_CHAR_GAP;
        s_k.deadline = now;
        break;

    default:
        s_k.state = KS_IDLE;
        break;
    }
}
