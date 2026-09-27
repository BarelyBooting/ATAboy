// Host test stand-in for the header CMakeLists.txt makes from ctl_build.h.in
// (the ATAboy Control API's build identity). The strings are the ones
// CMakeLists.txt sets for the API build; the hashes are fixed stand-ins.
#ifndef CTL_BUILD_H
#define CTL_BUILD_H
#if ATABOY_SAT && ATABOY_SAT_SMART_SAVES
#define CTL_FW_VERSION          "0.6f4-palimpsest-smartsaves"
#define CTL_FW_BANNER           "v0.6f4 (fork+api+smartsaves)"
#else
#define CTL_FW_VERSION          "0.6f4-palimpsest"
#define CTL_FW_BANNER           "v0.6f4 (fork+api)"
#endif
#define CTL_GIT                 "0123456789abcdef0123456789abcdef01234567"
#define CTL_DIRTY               0
#define CTL_SAT_POLICY_SHA256   "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
#define CTL_CTL_POLICY_SHA256   "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
#endif
