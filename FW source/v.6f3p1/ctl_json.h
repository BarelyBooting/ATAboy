// ATAboy Control, protocol v1 (palimpsest api/SPEC-v1.md): the JSON subset
// the firmware reads, and the writer for what it sends. Pure C, no SDK: the
// same file is compiled into the firmware and into the host tests and the
// parser fuzzer (the sat_policy.c pattern).
//
// What a request may be (SPEC-v1 section 3.2), everything else refused:
//  - one object at the top; values are objects (nesting depth at most 3,
//    counting the top), strings, integers, true, false. No arrays, no null,
//    no floats, exponents or signs, no leading zeros (0 itself is fine);
//    integers 0 to 2^53 - 1.
//  - strings of [A-Za-z0-9_.:-] only, at most 64 characters, no escapes;
//    keys the same, at most 24.
//  - spaces (0x20 only) anywhere between tokens.
//  - at most 32 keys, 32 scalar values and 32 objects in a line.
//  - nothing but spaces after the closing brace.
// A duplicate key in one object is CTLJ_E_DUP (E_SCHEMA to the client);
// anything else outside the grammar is CTLJ_E_PARSE.
//
// No heap, no recursion: the parse is a loop over the line with a stack of
// at most 3 open objects, into a fixed array of nodes the caller owns.
#ifndef CTL_JSON_H
#define CTL_JSON_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define CTLJ_MAX_DEPTH  3
#define CTLJ_MAX_KEYS   32
#define CTLJ_MAX_VALS   32
#define CTLJ_MAX_OBJS   32
#define CTLJ_MAX_STR    64
#define CTLJ_MAX_KEY    24
#define CTLJ_MAX_INT    9007199254740991ull     // 2^53 - 1

#define CTLJ_OK         0
#define CTLJ_E_PARSE    1
#define CTLJ_E_DUP      2

enum { CTLJ_OBJ = 1, CTLJ_STR, CTLJ_INT, CTLJ_BOOL };

// Node 0 is the top object (no key). Every other node is one member of an
// object: its key, its value, and the node of the object it belongs to.
typedef struct {
    uint8_t  type;          // CTLJ_*
    uint8_t  depth;         // of the object holding it; the top object is 1
    int8_t   parent;        // node index of that object; -1 for the top
    uint8_t  key_len;
    uint16_t key_off;       // offsets into the parsed line
    uint16_t str_off;       // CTLJ_STR: the characters between the quotes
    uint8_t  str_len;
    uint64_t num;           // CTLJ_INT: the value; CTLJ_BOOL: 0 or 1
} ctlj_node_t;

typedef struct {
    const char  *line;
    int          n;         // nodes used
    ctlj_node_t  node[1 + CTLJ_MAX_KEYS];
} ctlj_doc_t;

// Parse len bytes (the line without its \r\n). CTLJ_OK, CTLJ_E_PARSE or
// CTLJ_E_DUP; doc is valid only on CTLJ_OK.
int ctlj_parse(const char *line, size_t len, ctlj_doc_t *doc);

// The member of object node `obj` whose key is `key` (NUL-terminated), or -1.
int ctlj_find(const ctlj_doc_t *doc, int obj, const char *key);
// Members of object node `obj`.
int ctlj_members(const ctlj_doc_t *doc, int obj);
// Does node i's key equal key?
bool ctlj_key_is(const ctlj_doc_t *doc, int i, const char *key);
// Does string node i equal s?
bool ctlj_str_is(const ctlj_doc_t *doc, int i, const char *s);

// ---------------------------------------------------------------------------
//  The writer
// ---------------------------------------------------------------------------
// Appends to a fixed buffer. Keys and strings the firmware writes are its
// own ASCII texts; a byte outside 0x20-0x7E is written as '?' and a quote or
// backslash is escaped (\" and \\, the only escapes the firmware emits,
// SPEC-v1 3.1), so a line can never be broken by what goes into it. Drive
// bytes never go in as text: ctlw_hex writes them as lower-case hex. When the
// buffer is full, `overflow` is set and nothing more is written.
#define CTLW_MAX_DEPTH 8
typedef struct {
    char     *buf;
    uint32_t  cap, len;
    bool      overflow;
    uint8_t   depth;
    bool      first[CTLW_MAX_DEPTH + 1];
} ctlw_t;

void ctlw_init(ctlw_t *w, char *buf, uint32_t cap);
// key is NULL for the top object and for array elements.
void ctlw_obj(ctlw_t *w, const char *key);
void ctlw_arr(ctlw_t *w, const char *key);
void ctlw_end_obj(ctlw_t *w);
void ctlw_end_arr(ctlw_t *w);
void ctlw_str(ctlw_t *w, const char *key, const char *s);
void ctlw_strn(ctlw_t *w, const char *key, const char *s, size_t n);
void ctlw_uint(ctlw_t *w, const char *key, uint64_t v);
void ctlw_bool(ctlw_t *w, const char *key, bool v);
void ctlw_null(ctlw_t *w, const char *key);
void ctlw_hex(ctlw_t *w, const char *key, const uint8_t *p, size_t n);
// Raw bytes as they are (a stored JSON object, for job.last); the caller
// vouches that they are one whole value.
void ctlw_raw(ctlw_t *w, const char *key, const char *p, size_t n);

#endif
