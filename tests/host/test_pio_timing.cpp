// PIO bus timing against ATA PIO mode 0 (holistic review 2026-09-26, M-2
// and L-6). Reads ataboy.pio from the firmware tree, works out each strobe's
// timing from the instructions' side-set and delay values, and checks it
// against the mode 0 minimums with at least MARGIN_NS to spare. Also checks
// ide_pio_clkdiv_for (ide_pio_timing.h) keeps a state machine cycle at 100 ns
// or longer for the system clocks the SDK can run at.
//
//   test_pio_timing <path to ataboy.pio>
// Exit status is the number of failed checks.
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <regex>
#include "ide_pio_timing.h"

static int failures = 0, checks = 0;
#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; \
    printf("FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

// ATA PIO mode 0 (ATA/ATAPI-4 and later, table "PIO timing"), in ns.
static const int T0_CYCLE = 600, T2_8BIT = 290, T3_SETUP = 60, T4_HOLD = 30;
// A mode 0 device has 8-bit register data valid t2 - t5 after DIOR- falls.
static const int T5_SETUP = 50;
static const int MARGIN_NS = 100;

struct Instr { std::string op; int side; int delay; };

// The loop of one program: from label to the jmp back to it, inclusive.
static std::vector<Instr> loop_of(const std::string &text, const std::string &program, const std::string &label) {
    std::vector<Instr> out;
    std::istringstream in(text);
    std::string line;
    bool inprog = false, inloop = false;
    std::regex ins(R"(^\s+([a-z]+[^;]*?)\s+side\s+([01])(?:\s*\[(\d+)\])?\s*(;.*)?$)");
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.rfind(".program ", 0) == 0) { inprog = line == ".program " + program; inloop = false; continue; }
        if (!inprog) continue;
        if (line == label + ":") { inloop = true; continue; }
        if (!inloop) continue;
        std::smatch m;
        if (!std::regex_match(line, m, ins)) continue;
        Instr i{ m[1].str(), std::stoi(m[2].str()), m[3].matched ? std::stoi(m[3].str()) : 0 };
        out.push_back(i);
        if (i.op.rfind("jmp", 0) == 0) break;
    }
    return out;
}

static int cycles(const Instr &i) { return 1 + i.delay; }   // a WAIT takes one cycle when IORDY is high

int main(int argc, char **argv) {
    if (argc < 2) { printf("usage: test_pio_timing <ataboy.pio>\n"); return 99; }
    std::ifstream f(argv[1], std::ios::binary);
    if (!f) { printf("COULD NOT RUN: cannot open %s\n", argv[1]); return 98; }
    std::stringstream ss; ss << f.rdbuf();
    std::string text = ss.str();
    const int NS = (int)IDE_PIO_CYCLE_NS;

    // ---- read: ide_read, read_loop ----
    std::vector<Instr> rd = loop_of(text, "ide_read", "read_loop");
    CHECK(rd.size() >= 4, "read loop not found (%zu instructions)", rd.size());
    int low = 0, cyc = 0, sample = -1;
    for (auto &i : rd) {
        if (i.side == 0) {
            if (i.op.rfind("in pins", 0) == 0) sample = low * NS;
            low += cycles(i);
        }
        cyc += cycles(i);
    }
    int t2r = low * NS, t0r = cyc * NS;
    printf("read : DIOR- low %d ns, sampled at %d ns, cycle %d ns\n", t2r, sample, t0r);
    CHECK(t2r >= T2_8BIT + MARGIN_NS, "DIOR- %d ns, 8-bit minimum %d + %d", t2r, T2_8BIT, MARGIN_NS);
    CHECK(sample >= 0 && sample >= (T2_8BIT - T5_SETUP) + MARGIN_NS, "sampled at %d ns, data valid by %d + %d",
          sample, T2_8BIT - T5_SETUP, MARGIN_NS);
    CHECK(t0r >= T0_CYCLE + MARGIN_NS, "read cycle %d ns, minimum %d + %d", t0r, T0_CYCLE, MARGIN_NS);

    // ---- write: ide_write, write_loop ----
    std::vector<Instr> wr = loop_of(text, "ide_write", "write_loop");
    CHECK(wr.size() >= 5, "write loop not found (%zu instructions)", wr.size());
    int wlow = 0, wcyc = 0, setup = -1, hold = 0;
    bool after_out = false, strobed = false;
    for (auto &i : wr) {
        if (i.op.rfind("out pins", 0) == 0) { after_out = true; setup = 0; }
        if (i.side == 0) { strobed = true; wlow += cycles(i); }
        else if (after_out && !strobed) setup += cycles(i);
        else if (strobed) hold += cycles(i);           // high after the strobe, until the loop ends
        wcyc += cycles(i);
    }
    // Before the next word's out: the loop's first instruction (pull) too.
    for (auto &i : wr) { if (i.op.rfind("out pins", 0) == 0) break; hold += cycles(i); }
    int t2w = wlow * NS, t3 = setup * NS, t4 = hold * NS, t0w = wcyc * NS;
    printf("write: data setup %d ns, DIOW- low %d ns, hold %d ns, cycle %d ns\n", t3, t2w, t4, t0w);
    CHECK(t2w >= T2_8BIT + MARGIN_NS, "DIOW- %d ns, 8-bit minimum %d + %d", t2w, T2_8BIT, MARGIN_NS);
    CHECK(t3 >= T3_SETUP + MARGIN_NS, "data setup %d ns, minimum %d + %d", t3, T3_SETUP, MARGIN_NS);
    CHECK(t4 >= T4_HOLD + MARGIN_NS, "data hold %d ns, minimum %d + %d", t4, T4_HOLD, MARGIN_NS);
    CHECK(t0w >= T0_CYCLE + MARGIN_NS, "write cycle %d ns, minimum %d + %d", t0w, T0_CYCLE, MARGIN_NS);

    // ---- the divider (L-6) ----
    const uint32_t clocks[] = { 12000000, 48000000, 125000000, 133000000, 150000000, 200000000, 250000000, 300000000 };
    for (uint32_t hz : clocks) {
        uint32_t div = ide_pio_clkdiv_for(hz);
        double cycle_ns = 1e9 * div / hz;
        CHECK(cycle_ns >= NS - 1e-6, "%u Hz: divider %u gives %.1f ns a cycle", hz, div, cycle_ns);
        CHECK(cycle_ns < NS * 2 || div == 1, "%u Hz: divider %u gives %.1f ns, needlessly slow", hz, div, cycle_ns);
    }
    CHECK(ide_pio_clkdiv_for(150000000) == 15, "150 MHz: %u, was 15", ide_pio_clkdiv_for(150000000));

    printf("%d checks, %d failed\n", checks, failures);
    return failures > 255 ? 255 : failures;
}
