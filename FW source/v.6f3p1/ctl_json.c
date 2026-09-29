// ATAboy Control: the request parser and the response writer (ctl_json.h).
// Pure C: compiled unchanged into the firmware, the host tests and the
// fuzzer. Nothing here allocates, recurses or reads past `len`.

#include "ctl_json.h"
#include <string.h>

static bool tok_char(char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
           c == '_' || c == '.' || c == ':' || c == '-';
}

// A string token at p[*i] (the opening quote), at most max characters.
// On success *i is past the closing quote.
static bool read_str(const char *p, size_t len, size_t *i, size_t max, uint16_t *off, uint8_t *n) {
    size_t j = *i + 1, start = j;
    while (j < len && p[j] != '"') {
        if (!tok_char(p[j]) || j - start >= max) return false;     // escapes, spaces, anything else
        j++;
    }
    if (j >= len) return false;                                     // no closing quote
    *off = (uint16_t)start;
    *n = (uint8_t)(j - start);
    *i = j + 1;
    return true;
}

static bool same_key(const ctlj_doc_t *d, int a, int b) {
    return d->node[a].key_len == d->node[b].key_len &&
           memcmp(d->line + d->node[a].key_off, d->line + d->node[b].key_off, d->node[a].key_len) == 0;
}

int ctlj_parse(const char *p, size_t len, ctlj_doc_t *d) {
    enum { WANT_KEY_OR_END, WANT_KEY, WANT_COLON, WANT_VALUE, WANT_COMMA_OR_END, DONE };
    int stack[CTLJ_MAX_DEPTH];      // node index of each open object
    int depth = 0, state, keys = 0, vals = 0, objs = 1;
    int member = -1;                // the member whose value comes next
    size_t i = 0;
    d->line = p;
    d->n = 0;
    if (len > 0xFFFF) return CTLJ_E_PARSE;          // offsets are 16 bits (lines are 1 KB)
    while (i < len && p[i] == ' ') i++;
    if (i >= len || p[i] != '{') return CTLJ_E_PARSE;
    i++;
    memset(&d->node[0], 0, sizeof d->node[0]);
    d->node[0].type = CTLJ_OBJ;
    d->node[0].depth = 1;
    d->node[0].parent = -1;
    d->n = 1;
    stack[depth++] = 0;
    state = WANT_KEY_OR_END;
    while (state != DONE) {
        while (i < len && p[i] == ' ') i++;
        if (i >= len) return CTLJ_E_PARSE;          // the line ended inside the object
        char c = p[i];
        switch (state) {
        case WANT_KEY_OR_END:
        case WANT_KEY:
            if (c == '}' && state == WANT_KEY_OR_END) { i++; goto close; }
            if (c != '"') return CTLJ_E_PARSE;
            if (++keys > CTLJ_MAX_KEYS) return CTLJ_E_PARSE;
            member = d->n++;
            memset(&d->node[member], 0, sizeof d->node[member]);
            d->node[member].parent = (int8_t)stack[depth - 1];
            d->node[member].depth = (uint8_t)depth;
            if (!read_str(p, len, &i, CTLJ_MAX_KEY, &d->node[member].key_off, &d->node[member].key_len))
                return CTLJ_E_PARSE;
            state = WANT_COLON;
            break;
        case WANT_COLON:
            if (c != ':') return CTLJ_E_PARSE;
            i++;
            state = WANT_VALUE;
            break;
        case WANT_VALUE: {
            ctlj_node_t *m = &d->node[member];
            if (c == '{') {
                if (depth >= CTLJ_MAX_DEPTH || ++objs > CTLJ_MAX_OBJS) return CTLJ_E_PARSE;
                m->type = CTLJ_OBJ;
                stack[depth++] = member;
                i++;
                state = WANT_KEY_OR_END;
                break;
            }
            if (++vals > CTLJ_MAX_VALS) return CTLJ_E_PARSE;
            if (c == '"') {
                m->type = CTLJ_STR;
                if (!read_str(p, len, &i, CTLJ_MAX_STR, &m->str_off, &m->str_len)) return CTLJ_E_PARSE;
            } else if (c >= '0' && c <= '9') {
                uint64_t v = 0;
                size_t start = i;
                while (i < len && p[i] >= '0' && p[i] <= '9') {
                    // 2^53 - 1 has 16 digits: 17 is always too many, and the
                    // value is checked before every digit is added.
                    if (i - start >= 16) return CTLJ_E_PARSE;
                    v = v * 10 + (uint64_t)(p[i] - '0');
                    i++;
                }
                if (p[start] == '0' && i - start > 1) return CTLJ_E_PARSE;     // leading zero
                if (v > CTLJ_MAX_INT) return CTLJ_E_PARSE;
                m->type = CTLJ_INT;
                m->num = v;
            } else if (len - i >= 4 && memcmp(p + i, "true", 4) == 0) {
                m->type = CTLJ_BOOL; m->num = 1; i += 4;
            } else if (len - i >= 5 && memcmp(p + i, "false", 5) == 0) {
                m->type = CTLJ_BOOL; m->num = 0; i += 5;
            } else {
                return CTLJ_E_PARSE;                // null, arrays, signs, anything else
            }
            // What follows a scalar must be a separator (so 1.5, 1e3, truex fail here).
            if (i < len && p[i] != ' ' && p[i] != ',' && p[i] != '}') return CTLJ_E_PARSE;
            state = WANT_COMMA_OR_END;
            break;
        }
        case WANT_COMMA_OR_END:
            if (c == ',') { i++; state = WANT_KEY; break; }
            if (c != '}') return CTLJ_E_PARSE;
            i++;
        close:
            depth--;
            if (depth == 0) { state = DONE; break; }
            state = WANT_COMMA_OR_END;
            break;
        }
    }
    while (i < len && p[i] == ' ') i++;
    if (i != len) return CTLJ_E_PARSE;              // trailing garbage
    // Duplicate keys, object by object (SPEC-v1 3.2: E_SCHEMA, not E_PARSE).
    for (int a = 1; a < d->n; a++)
        for (int b = a + 1; b < d->n; b++)
            if (d->node[a].parent == d->node[b].parent && same_key(d, a, b)) return CTLJ_E_DUP;
    return CTLJ_OK;
}

bool ctlj_key_is(const ctlj_doc_t *d, int i, const char *key) {
    size_t n = strlen(key);
    return i > 0 && i < d->n && d->node[i].key_len == n && memcmp(d->line + d->node[i].key_off, key, n) == 0;
}

bool ctlj_str_is(const ctlj_doc_t *d, int i, const char *s) {
    size_t n = strlen(s);
    return i > 0 && i < d->n && d->node[i].type == CTLJ_STR && d->node[i].str_len == n &&
           memcmp(d->line + d->node[i].str_off, s, n) == 0;
}

int ctlj_find(const ctlj_doc_t *d, int obj, const char *key) {
    for (int i = 1; i < d->n; i++)
        if (d->node[i].parent == obj && ctlj_key_is(d, i, key)) return i;
    return -1;
}

int ctlj_members(const ctlj_doc_t *d, int obj) {
    int n = 0;
    for (int i = 1; i < d->n; i++) if (d->node[i].parent == obj) n++;
    return n;
}

// ---------------------------------------------------------------------------
//  The writer
// ---------------------------------------------------------------------------

static void put(ctlw_t *w, char c) {
    if (w->overflow) return;
    if (w->len >= w->cap) { w->overflow = true; return; }
    w->buf[w->len++] = c;
}

static void put_text(ctlw_t *w, const char *s, size_t n) {
    for (size_t i = 0; i < n; i++) {
        char c = s[i];
        if (c == '"' || c == '\\') { put(w, '\\'); put(w, c); }
        else put(w, (c >= 0x20 && c <= 0x7E) ? c : '?');
    }
}

void ctlw_init(ctlw_t *w, char *buf, uint32_t cap) {
    w->buf = buf; w->cap = cap; w->len = 0; w->overflow = false; w->depth = 0;
    w->first[0] = true;
}

static void member(ctlw_t *w, const char *key) {
    if (!w->first[w->depth]) put(w, ',');
    w->first[w->depth] = false;
    if (key) { put(w, '"'); put_text(w, key, strlen(key)); put(w, '"'); put(w, ':'); }
}

static void w_open(ctlw_t *w, const char *key, char c) {
    member(w, key);
    put(w, c);
    if (w->depth < CTLW_MAX_DEPTH) w->depth++;
    else w->overflow = true;
    w->first[w->depth] = true;
}

void ctlw_obj(ctlw_t *w, const char *key) { w_open(w, key, '{'); }
void ctlw_arr(ctlw_t *w, const char *key) { w_open(w, key, '['); }
void ctlw_end_obj(ctlw_t *w) { put(w, '}'); if (w->depth) w->depth--; }
void ctlw_end_arr(ctlw_t *w) { put(w, ']'); if (w->depth) w->depth--; }

void ctlw_strn(ctlw_t *w, const char *key, const char *s, size_t n) {
    member(w, key); put(w, '"'); put_text(w, s, n); put(w, '"');
}
void ctlw_str(ctlw_t *w, const char *key, const char *s) { ctlw_strn(w, key, s, strlen(s)); }

void ctlw_uint(ctlw_t *w, const char *key, uint64_t v) {
    char t[20];
    int n = 0;
    do { t[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    member(w, key);
    while (n) put(w, t[--n]);
}

void ctlw_bool(ctlw_t *w, const char *key, bool v) {
    member(w, key);
    const char *s = v ? "true" : "false";
    while (*s) put(w, *s++);
}

void ctlw_null(ctlw_t *w, const char *key) {
    member(w, key); put(w, 'n'); put(w, 'u'); put(w, 'l'); put(w, 'l');
}

void ctlw_hex(ctlw_t *w, const char *key, const uint8_t *p, size_t n) {
    static const char hx[] = "0123456789abcdef";
    member(w, key);
    put(w, '"');
    for (size_t i = 0; i < n; i++) { put(w, hx[p[i] >> 4]); put(w, hx[p[i] & 15]); }
    put(w, '"');
}

void ctlw_raw(ctlw_t *w, const char *key, const char *p, size_t n) {
    if (n == 0) { ctlw_null(w, key); return; }      // never a key with no value (review A LOW-5)
    member(w, key);
    for (size_t i = 0; i < n; i++) put(w, p[i]);
}
