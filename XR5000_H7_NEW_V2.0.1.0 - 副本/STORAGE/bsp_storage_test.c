/**
 * @file    bsp_storage_test.c
 * @brief   ï¿½ï¿½Ï»ï¿½ï¿½È«ï¿½ï¿½ï¿½Ì¶ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½Ô¶ï¿½ï¿½ï¿½ï¿½ï¿½Ä£ï¿½ï¿½ - ï¿½×¶Î±ï¿½ï¿½ï¿½ï¿½ï¿½×´Ì¬ï¿½ï¿½(ï¿½ï¿½ï¿½Ô·ï¿½ï¿½ï¿½v2 ï¿½ï¿½1.3)
 * @details ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½(Ô¼5ï¿½ï¿½ï¿½ï¿½): S_IDLEï¿½ï¿½ï¿½ï¿½Â·ï¿½È¶ï¿½ -> S1ï¿½ï¿½Í¨ï¿½Â¼ï¿½x10(2sï¿½ï¿½ï¿?
 *          -> S2ï¿½ï¿½ï¿½ï¿½/ï¿½Ö¸ï¿½ï¿½É¶ï¿½ -> S3ï¿½×¾ï¿½(Ã¿ï¿½ï¿½ï¿½ï¿½Î¨Ò»0x02) -> S4ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½x6(5sï¿½ï¿½ï¿?
 *          -> S5ï¿½ç±©40ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×¢ï¿½ï¿½(ï¿½ï¿½ï¿½ï¿½32ï¿½ï¿½, ï¿½ï¿½ï¿½ï¿½ï¿?ï¿½ï¿½2) -> S6ï¿½ï¿½Î»(ï¿½ï¿½ï¿½×¾ï¿½ï¿½ï¿½Ö¾)
 *          -> S7ï¿½ï¿½Î»ï¿½ï¿½ï¿½Ù»ï¿½(ï¿½ï¿½Ö¤0x02ï¿½ï¿½ï¿½ï¿½) -> S8ï¿½ï¿½Ä¬180s -> Ñ­ï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½.
 *          ï¿½×¶Î±ï¿½Ç¾ï¿?StorageTx_SendTestLog Í¸ï¿½ï¿½, ï¿½æ´¢ï¿½ï¿½USB CDCÔ­ï¿½ï¿½ï¿½ï¿½ï¿?
 */
#include "bsp_storage_test.h"
#if STX_TEST_ENABLE
#include <stdio.h>
#include <string.h>
#include "bsp_storage_tx.h"
#include "bsp_storage_event.h"

/* ---------- ï¿½×¶Î¶ï¿½ï¿½ï¿½ ---------- */
enum {
    S_IDLE = 0, S1_BASIC, S2_FAULT, S3_FIRST, S4_FIRE, S5_STORM,
    S6_RESET, S7_SECOND_FIRST, S8_SILENCE, S_CYCLE_END
};

static uint8_t  s_stage = S_IDLE;
static uint32_t s_tick  = 0;      /* 100msÈ«ï¿½ï¿½Ê±ï¿½ï¿½(ï¿½ï¿½ï¿½Î¿ï¿½, ï¿½×¶ï¿½ï¿½Ú¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½) */
static uint32_t s_stage_tick = 0; /* ï¿½ï¿½Ç°ï¿½×¶ï¿½ï¿½Ú½ï¿½ï¿½ï¿½ */
static uint8_t  s_cnt = 0;        /* ï¿½×¶ï¿½ï¿½ï¿½×¢ï¿½ï¿½ï¿½ï¿½ï¿?S1/S4ï¿½ï¿½, ï¿½Ð½×¶ï¿½Ê±ï¿½ï¿½ï¿½ï¿½) */
static uint32_t s_cycle = 0;
static uint8_t  s_rst_logged = 0;  /* ¸´Î»Ô­ÒòÖ»´òÒ»´Î */      /* ï¿½ï¿½ï¿½ï¿½ï¿½Ö´ï¿½ */
static uint32_t s_storm_injected = 0;  /* S5ï¿½ï¿½×¢ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */

/* ---------- ï¿½ï¿½ï¿½ï¿½ ---------- */
/* ï¿½ï¿½ï¿½Í½×¶Î±ï¿½ï¿?ï¿½ï¿½0x07Í¸ï¿½ï¿½, ï¿½æ´¢ï¿½ï¿½USB×ªï¿½ï¿½, ï¿½ï¿½Ð´Flash) */
static void TestLog(const char *msg)
{
    char buf[80];
    snprintf(buf, sizeof(buf), "[T] cycle=%lu %s", (unsigned long)s_cycle, msg);
    (void)StorageTx_SendTestLog(buf);
}

/* ---------- ÖÜÆÚ×¢Èë(LITE): ÉÏµç3sÆðÃ¿ÖÜÆÚ21²½22Ìõ, ÀäÈ´60sºó×Ô¶¯ÖØ×¢ ---------- */
/* ¸²¸ÇÈ«²¿22¸öÊÂ¼þÂë: Z0Ê×¾¯(evt2) + Z1»ð¾¯(evt3) + Z2¹ÊÕÏ(evt80/100) +
 * Z3ÆÕÍ¨18ÖÖ(26/72/73/125/126/19/29/122/120/121/128/129/130/131/123/124/70/71);
 * ¹©GB4717 0x40Ð­Òéµ¼³öÖðÌõºË¶Ô, ÎÞÐèÉÏµç¸´Î»/ÈË¹¤¸ÉÔ¤ */
static uint8_t  s_lite_stage = 0;   /* 0=´ý»ú 1=×¢ÈëÖÐ 2=ÀäÈ´(µ¹¼ÆÊ±ºóÖØ×¢) */
static uint8_t  s_lite_step  = 0;
static uint16_t s_lite_tick  = 0;

static void LiteInject(uint8_t i)
{
    switch (i) {
    case 0:  TestLog("LITE 0/20 fire+first evt2/3"); StorageEvent_LogFire(5, DEV_TYPE_TEMPERATURE, 1, 0); break;
    case 1:  TestLog("LITE 1/20 fault evt80");       StorageEvent_LogFault(6, DEV_TYPE_SMOKE, 1, 0, 0); break;
    case 2:  TestLog("LITE 2/20 faultrec evt100");   StorageEvent_LogFault(6, DEV_TYPE_SMOKE, 1, 0, 1); break;
    case 3:  TestLog("LITE 3/20 feedback evt26");    StorageEvent_LogFeedback(8, DEV_TYPE_CONTROL_DEV, 0); break;
    case 4:  TestLog("LITE 4/20 shield evt72");      StorageEvent_LogShield(7, DEV_TYPE_SMOKE, 0); break;
    case 5:  TestLog("LITE 5/20 shieldrel evt73");   StorageEvent_LogShield(7, DEV_TYPE_SMOKE, 1); break;
    case 6:  TestLog("LITE 6/20 manual evt125");     StorageEvent_LogManualAuto(17, 1); break;   /* 17=SYS_HAND_AUTO_Package_ID */
    case 7:  TestLog("LITE 7/20 auto evt126");       StorageEvent_LogManualAuto(17, 0); break;
    case 8:  TestLog("LITE 8/20 start evt19");       StorageEvent_LogStart(9, DEV_TYPE_CONTROL_DEV); break;
    case 9:  TestLog("LITE 9/20 stop evt29");        StorageEvent_LogLinkageAction(10, 1, 0); break;
    case 10: TestLog("LITE 10/20 reset evt122");     StorageEvent_LogReset(); break;
    case 11: TestLog("LITE 11/20 poweron evt120");   StorageEvent_LogPowerOn(); break;
    case 12: TestLog("LITE 12/20 poweroff evt121");  StorageEvent_LogPowerOff(); break;
    case 13: TestLog("LITE 13/20 confirm evt128");   StorageEvent_LogConfirmButton(); break;
    case 14: TestLog("LITE 14/20 check evt129");     StorageEvent_LogCheckButton(); break;
    case 15: TestLog("LITE 15/20 linkbtn evt130");   StorageEvent_LogLinkageStartButton(11, DEV_TYPE_CONTROL_DEV); break;
    case 16: TestLog("LITE 16/20 clock evt131");     StorageEvent_LogClockAdjust(); break;
    case 17: TestLog("LITE 17/20 selfcheck evt123"); StorageEvent_LogSelfCheck(0); break;
    case 18: TestLog("LITE 18/20 selffail evt124");  StorageEvent_LogSelfCheck(1); break;
    case 19: TestLog("LITE 19/20 supervise evt70");  StorageEvent_LogSupervise(12, DEV_TYPE_MULTI_SENSOR, 0); break;
    case 20: TestLog("LITE 20/20 supervrel evt71");  StorageEvent_LogSupervise(12, DEV_TYPE_MULTI_SENSOR, 1); break;
    default: break;
    }
}

#define LITE_COOLDOWN_TICKS  600U   /* ÀäÈ´60s(100ms/tick): >´æ´¢²à²Á³ý15s+ÓàÁ¿, ±£Ö¤½Å±¾ÔÚÀäÈ´ÆÚÄÚ²Á³ý²»¿ç×¢Èë´° */

static void LiteTest_Tick(void)
{
    s_lite_tick++;
    switch (s_lite_stage) {
    case 0:                    /* ÉÏµçµÈ3s, µÈ×ÜÏßÓë·¢ËÍÁ´Â·ÎÈ¶¨ */
        if (s_lite_tick >= 30U) {
            s_lite_stage = 1; s_lite_tick = 0; s_lite_step = 0;
            TestLog("LITE CYCLE begin (21 steps)");
        }
        break;
    case 1:                    /* 500msÒ»²½, 21²½¹²22ÌõÔ¼10.5s */
        if (s_lite_tick >= 5U) {
            s_lite_tick = 0;
            LiteInject(s_lite_step);
            s_lite_step++;
            if (s_lite_step >= 21U) {
                s_lite_stage = 2; s_lite_tick = 0;
                TestLog("LITE CYCLE DONE (22 recs)");
            }
        }
        break;
    case 2:                    /* ÀäÈ´60s: ÇåÊ×¾¯±êÖ¾ºó×Ô¶¯ÖØ×¢, ÖÜÆÚÔËÐÐÎÞÐèÈË¹¤¸ÉÔ¤ */
    default:
        if (s_lite_tick >= LITE_COOLDOWN_TICKS) {
            s_lite_tick = 0; s_lite_step = 0;
            StorageEvent_ResetFirstFire();   /* ±£Ö¤ÏÂÖÜÆÚstep0È·¶¨ÔÙÉúÊ×¾¯evt2 */
            s_cycle++;
            s_lite_stage = 1;
            TestLog("LITE CYCLE begin (21 steps)");
        }
        break;
    }
}

/* S1: ï¿½ï¿½ï¿½ï¿½×¢ï¿½ï¿½ï¿½ï¿½Í¨ï¿½ï¿½ï¿½ï¿½ï¿½Â¼ï¿½(ï¿½ï¿½ï¿½ï¿½/ï¿½ï¿½ï¿½ï¿½/ï¿½Ô¼ï¿½ -> 0x01ï¿½ï¿½Í¨ï¿½ï¿½ï¿½ï¿½) */
static void InjectNormal(uint8_t i)
{
    switch (i % 3U) {
    case 0: StorageEvent_LogFeedback(20 + i, DEV_TYPE_CONTROL_DEV, 0x0101); break;
    case 1: StorageEvent_LogShield(21 + i, DEV_TYPE_SMOKE, 0);              break;
    default: StorageEvent_LogSelfCheck(0);                                  break;
    }
}

/* ---------- ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½(TaskLoopï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ã¿100msï¿½ï¿½ï¿½ï¿½Ò»ï¿½ï¿½) ---------- */
void StorageTest_Tick(void)
{
    LiteTest_Tick();            /* LITEÖÜÆÚ×¢ÈëÈ«³Ì½Ó¹Ü: S1~S8·ç±©´óÑ­»·±£Áô±àÒëµ«ÓÀ²»½øÈë */
    return;                     /* (ÈçÐè»Ö¸´Ô­S1~S8´óÑ­»·, É¾³ý±¾ÐÐreturn) */
    s_tick++;
    s_stage_tick++;

    switch (s_stage) {

    case S_IDLE:   /* µÈ´ý2s: ÉÏµçºó×ÜÏßÎÈ¶¨ */
        if (!s_rst_logged) {   /* Ê×´Î½øÈë¿ÕÏÐ: ¾­0x07Í¸´«¸´Î»Ô­Òò(¶¨Î»²âÊÔÆÚ¸´Î») */
            const char *rs = "UNK";   /* H7: ¸´Î»±êÖ¾ÔÚRSR, ÓÃHALºê¶Á */
            if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST))      rs = "IWDG";
            else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDG1RST)) rs = "WWDG";
            else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))   rs = "SOFT";
            else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))   rs = "BOR";
            else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))   rs = "POR";
            else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))   rs = "PIN";
            __HAL_RCC_CLEAR_RESET_FLAGS();
            char msg[40];
            snprintf(msg, sizeof(msg), "RESET CAUSE=%s", rs);
            TestLog(msg);
            s_rst_logged = 1;
        }

        if (s_stage_tick >= 20U) {
            s_stage = S1_BASIC; s_stage_tick = 0; s_cnt = 0;
            TestLog("S1 BASIC begin (feedback/shield/selfcheck -> zone3)");
        }
        break;

    case S1_BASIC: /* 200msÒ»Ìõ x 10Ìõ(½×¶ÎÄÚ¾ùÔÈ¼ä¸ô, ±£Ö¤Ç¡ºÃ10Ìõ) */
        if (s_stage_tick >= 2U) {
            s_stage_tick = 0;
            InjectNormal(s_cnt++);
            if (s_cnt >= 10U) {
                s_stage = S2_FAULT; s_stage_tick = 0;
                TestLog("S1 end (expect 10x cmd=01 zone3)");
            }
        }
        break;

    case S2_FAULT: /* t=1.5s¹ÊÕÏ, t=3s»Ö¸´(80/100³É¶ÔÈë0x04¹ÊÕÏÇø) */
        if (s_stage_tick == 15U) {
            StorageEvent_LogFault(5, DEV_TYPE_MULTI_SENSOR, 1, 0, 0);
            TestLog("S2 fault occur (cmd=04 zone2)");
        }
        if (s_stage_tick >= 30U) {
            StorageEvent_LogFault(5, DEV_TYPE_MULTI_SENSOR, 1, 0, 1);
            TestLog("S2 fault recover (pair complete)");
            s_stage = S3_FIRST; s_stage_tick = 0;
        }
        break;

    case S3_FIRST: /* Ê×¾¯: È«ÖÜÆÚµÚÒ»´Î0x02 */
        if (s_stage_tick >= 30U) {
            StorageEvent_LogFire(5, DEV_TYPE_TEMPERATURE, 1, 0);
            TestLog("S3 FIRST FIRE (cmd=02 zone0, once per cycle)");
            s_stage = S4_FIRE; s_stage_tick = 0; s_cnt = 0;
        }
        break;

    case S4_FIRE:  /* 500msÒ»Ìõ x 6ÌõÄ£Äâ»ð¾¯(½×¶ÎÄÚ¾ùÔÈ¼ä¸ô, ±£Ö¤Ç¡ºÃ6Ìõ) */
        if (s_stage_tick >= 5U) {
            s_stage_tick = 0;
            StorageEvent_LogFire((uint8_t)(6 + s_cnt), DEV_TYPE_SMOKE, 1, 0);
            s_cnt++;
            if (s_cnt >= 6U) {
                s_stage = S5_STORM; s_stage_tick = 0; s_storm_injected = 0;
                TestLog("S5 STORM begin (40x fire in one tick)");
            }
        }
        break;

    case S5_STORM: /* ·ç±©: ½×¶Î¿ªÊ¼Ò»´Î×¢Èë40Ìõ(¶ÓÁÐ´æ32Ìõ, ×î¾É8Ìõ±»¼·µô) */
        while (s_storm_injected < 40U) {
            StorageEvent_LogFire((uint8_t)(30 + s_storm_injected), DEV_TYPE_SMOKE, 1, 0);
            s_storm_injected++;
        }
        /* µÈ´ý·¢ËÍÁ´Â·¾¡¿ìÅÅ¿Õ; 3sºó½áÊø */
        if (s_stage_tick >= 30U) {
            TestLog("S5 end (expect 32ï¿½ï¿½2x cmd=03, oldest dropped; M1: unplug PB6/PB7 NOW)");
            s_stage = S6_RESET; s_stage_tick = 0;
        }
        break;

    case S6_RESET: /* ¸´Î»: ÇåÊ×¾¯±êÖ¾+¼ÇÂ¼¸´Î»ÊÂ¼þ */
        if (s_stage_tick >= 20U) {
            StorageEvent_ResetFirstFire();
            StorageEvent_LogReset();
            TestLog("S6 RESET (EVT_RESET=122 zone3, first-fire flag cleared)");
            s_stage = S7_SECOND_FIRST; s_stage_tick = 0;
        }
        break;

    case S7_SECOND_FIRST: /* ¸´Î»ºóÔÙ¼ì: Ó¦ÔÙ´Î²úÉú0x02 */
        if (s_stage_tick >= 30U) {
            StorageEvent_LogFire(7, DEV_TYPE_TEMPERATURE, 1, 0);
            TestLog("S7 SECOND FIRST FIRE (cmd=02 AGAIN after reset)");
            s_stage = S8_SILENCE; s_stage_tick = 0;
            TestLog("S8 SILENCE 30s begin");
        }
        break;

    case S8_SILENCE: /* ¾²Ä¬30s: ×ÜÏßÎÞ¼ÇÂ¼(ÑéÖ¤ÎÞÐé¼ÙÐ´Èë; Ò²¸øS5·ç±©ÅÅ¿Õ, ¹©M1) */
        if (s_stage_tick >= 300U) {
            s_stage = S_CYCLE_END;
        }
        break;

    case S_CYCLE_END:
    default:
        TestLog("CYCLE END. >>> replug, power-cycle, USB export compare <<<");
        StorageEvent_ResetFirstFire();  /* N3: clear first-fire flag at cycle boundary, ensure next cycle S3 sends 0x02 */
        s_cycle++;
        s_stage = S_IDLE; s_stage_tick = 0;  /* loop to next cycle */
        break;
    }
}

/* ---------- ï¿½ï¿½Ê¼ï¿½ï¿½(StorageTx_Init()Î²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½) ---------- */
void StorageTest_Init(void)
{
    s_stage = S_IDLE; s_tick = 0; s_stage_tick = 0; s_cycle = 1;
    /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö¾ï¿½ï¿½ï¿½×¸ï¿½Tickï¿½ï¿½(ï¿½ï¿½Ê±TaskLoopï¿½ï¿½È·ï¿½Ï³ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½ï¿? */
}
#endif /* STX_TEST_ENABLE */
