/*
 * RISC-V translation routines for the Zicldst Experimental Extension.
 *
 * Copyright (c) 2024 Yangyu Chen
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define REQUIRE_ZICLDST(ctx) do {       \
    if (!ctx->cfg_ptr->ext_zicldst) {   \
        return false;                   \
    }                                   \
} while (0)

static bool gen_cload(DisasContext *ctx, arg_clb *a, MemOp memop)
{
    TCGLabel *l = gen_new_label();
    TCGv base = get_gpr(ctx, a->rs1, EXT_SIGN);
    target_ulong orig_pc_save = ctx->pc_save;
    bool out;

    tcg_gen_addi_i64(cload_cnt, cload_cnt, 1);

    if (get_xl(ctx) == MXL_RV128) {
        TCGv baseh = get_gprh(ctx, a->rs1);
        TCGv tmp = tcg_temp_new();
        TCGCond cond = gen_compare_i128(true, tmp, base, baseh, 0, 0,
                                        TCG_COND_EQ);
        tcg_gen_brcondi_tl(cond, tmp, 0, l);
    } else {
        tcg_gen_brcondi_tl(TCG_COND_EQ, base, 0, l);
    }
    out = gen_load(ctx, a, memop);
    tcg_gen_addi_i64(cload_taken_cnt, cload_taken_cnt, 1);
    gen_goto_tb(ctx, 1, ctx->cur_insn_len);
    ctx->pc_save = orig_pc_save;
    gen_set_label(l); /* branch taken */
    gen_set_gpri(ctx, a->rd, 0);

    return out;
}

static bool gen_cstore(DisasContext *ctx, arg_csb *a, MemOp memop)
{
    TCGLabel *l = gen_new_label();
    TCGv base = get_gpr(ctx, a->rs1, EXT_SIGN);
    bool out;

    tcg_gen_addi_i64(cstore_cnt, cstore_cnt, 1);

    if (get_xl(ctx) == MXL_RV128) {
        TCGv baseh = get_gprh(ctx, a->rs1);
        TCGv tmp = tcg_temp_new();
        TCGCond cond = gen_compare_i128(true, tmp, base, baseh, 0, 0,
                                        TCG_COND_EQ);
        tcg_gen_brcondi_tl(cond, tmp, 0, l);
    } else {
        tcg_gen_brcondi_tl(TCG_COND_EQ, base, 0, l);
    }
    out = gen_store(ctx, a, memop);
    tcg_gen_addi_i64(cstore_taken_cnt, cstore_taken_cnt, 1);
    gen_set_label(l);

    return out;
}

static bool trans_clb(DisasContext *ctx, arg_clb *a)
{
    REQUIRE_ZICLDST(ctx);
    return gen_cload(ctx, a, MO_SB);
}

static bool trans_clh(DisasContext *ctx, arg_clh *a)
{
    REQUIRE_ZICLDST(ctx);
    return gen_cload(ctx, a, MO_TESW);
}

static bool trans_clw(DisasContext *ctx, arg_clw *a)
{
    REQUIRE_ZICLDST(ctx);
    return gen_cload(ctx, a, MO_TESL);
}

static bool trans_cld(DisasContext *ctx, arg_cld *a)
{
    REQUIRE_64_OR_128BIT(ctx);
    REQUIRE_ZICLDST(ctx);
    return gen_cload(ctx, a, MO_TESQ);
}

static bool trans_clbu(DisasContext *ctx, arg_clbu *a)
{
    REQUIRE_ZICLDST(ctx);
    return gen_cload(ctx, a, MO_UB);
}

static bool trans_clhu(DisasContext *ctx, arg_clhu *a)
{
    REQUIRE_ZICLDST(ctx);
    return gen_cload(ctx, a, MO_TEUW);
}

static bool trans_clwu(DisasContext *ctx, arg_clwu *a)
{
    REQUIRE_64_OR_128BIT(ctx);
    REQUIRE_ZICLDST(ctx);
    return gen_cload(ctx, a, MO_TEUL);
}

static bool trans_csb(DisasContext *ctx, arg_csb *a)
{
    REQUIRE_ZICLDST(ctx);
    return gen_cstore(ctx, a, MO_SB);
}

static bool trans_csh(DisasContext *ctx, arg_csh *a)
{
    REQUIRE_ZICLDST(ctx);
    return gen_cstore(ctx, a, MO_TESW);
}

static bool trans_csw(DisasContext *ctx, arg_csw *a)
{
    REQUIRE_ZICLDST(ctx);
    return gen_cstore(ctx, a, MO_TESL);
}

static bool trans_csd(DisasContext *ctx, arg_csd *a)
{
    REQUIRE_64_OR_128BIT(ctx);
    REQUIRE_ZICLDST(ctx);
    return gen_cstore(ctx, a, MO_TEUQ);
}

