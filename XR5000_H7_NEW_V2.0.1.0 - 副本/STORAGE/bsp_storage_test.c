/**
 * @file    bsp_storage_test.c
 * @brief   ��ϻ��ȫ���̶�ʱ�����Զ�����ģ�� - �׶α�����״̬��(���Է���v2 ��1.3)
 * @details ��������(Լ5����): S_IDLE����·�ȶ� -> S1��ͨ�¼�x10(2s���)
 *          -> S2����/�ָ��ɶ� -> S3�׾�(ÿ����Ψһ0x02) -> S4������x6(5s���)
 *          -> S5�籩40������ע��(����32��, �����8��2) -> S6��λ(���׾���־)
 *          -> S7��λ���ٻ�(��֤0x02����) -> S8��Ĭ180s -> ѭ����һ��.
 *          �׶α�Ǿ� StorageTx_SendTestLog ͸��, �洢��USB CDCԭ�����.
 */
#include "bsp_storage_test.h"
#if STX_TEST_ENABLE
#include <stdio.h>
#include <string.h>
#include "bsp_storage_tx.h"
#include "bsp_storage_event.h"

/* ---------- �׶ζ��� ---------- */
enum {
    S_IDLE = 0, S1_BASIC, S2_FAULT, S3_FIRST, S4_FIRE, S5_STORM,
    S6_RESET, S7_SECOND_FIRST, S8_SILENCE, S_CYCLE_END
};

static uint8_t  s_stage = S_IDLE;
static uint32_t s_tick  = 0;      /* 100msȫ��ʱ��(���ο�, �׶��ڼ���������) */
static uint32_t s_stage_tick = 0; /* ��ǰ�׶��ڽ��� */
static uint8_t  s_cnt = 0;        /* �׶���ע�����(S1/S4��, �н׶�ʱ����) */
static uint32_t s_cycle = 0;      /* �����ִ� */
static uint32_t s_storm_injected = 0;  /* S5��ע������ */

/* ---------- ���� ---------- */
/* ���ͽ׶α��(��0x07͸��, �洢��USBת��, ��дFlash) */
static void TestLog(const char *msg)
{
    char buf[80];
    snprintf(buf, sizeof(buf), "[T] cycle=%lu %s", (unsigned long)s_cycle, msg);
    (void)StorageTx_SendTestLog(buf);
}

/* S1: ����ע����ͨ�����¼�(����/����/�Լ� -> 0x01��ͨ����) */
static void InjectNormal(uint8_t i)
{
    switch (i % 3U) {
    case 0: StorageEvent_LogFeedback(20 + i, DEV_TYPE_CONTROL_DEV, 0x0101); break;
    case 1: StorageEvent_LogShield(21 + i, DEV_TYPE_SMOKE, 0);              break;
    default: StorageEvent_LogSelfCheck(0);                                  break;
    }
}

/* ---------- ������(TaskLoop������ÿ100ms����һ��) ---------- */
void StorageTest_Tick(void)
{
    s_tick++;
    s_stage_tick++;

    switch (s_stage) {

    case S_IDLE:   /* �������10s����·�ȶ� */
        if (s_stage_tick >= 100U) {
            s_stage = S1_BASIC; s_stage_tick = 0; s_cnt = 0;
            TestLog("S1 BASIC begin (feedback/shield/selfcheck -> zone3)");
        }
        break;

    case S1_BASIC: /* 2sһ�� x 10��(�׶��ڼ�����, ��֤ǡ��10��) */
        if (s_stage_tick >= 20U) {
            s_stage_tick = 0;
            InjectNormal(s_cnt++);
            if (s_cnt >= 10U) {
                s_stage = S2_FAULT; s_stage_tick = 0;
                TestLog("S1 end (expect 10x cmd=01 zone3)");
            }
        }
        break;

    case S2_FAULT: /* t=35s����, t=40s�ָ�(80/100�ɶ���0x04������) */
        if (s_stage_tick == 50U) {
            StorageEvent_LogFault(5, DEV_TYPE_MULTI_SENSOR, 1, 0, 0);
            TestLog("S2 fault occur (cmd=04 zone2)");
        }
        if (s_stage_tick >= 100U) {
            StorageEvent_LogFault(5, DEV_TYPE_MULTI_SENSOR, 1, 0, 1);
            TestLog("S2 fault recover (pair complete)");
            s_stage = S3_FIRST; s_stage_tick = 0;
        }
        break;

    case S3_FIRST: /* �׾�: �����ڵ�һ��0x02 */
        if (s_stage_tick >= 100U) {
            StorageEvent_LogFire(5, DEV_TYPE_TEMPERATURE, 1, 0);
            TestLog("S3 FIRST FIRE (cmd=02 zone0, once per cycle)");
            s_stage = S4_FIRE; s_stage_tick = 0; s_cnt = 0;
        }
        break;

    case S4_FIRE:  /* 5sһ�� x 6��������(�׶��ڼ���, ��֤ǡ��6��) */
        if (s_stage_tick >= 50U) {
            s_stage_tick = 0;
            StorageEvent_LogFire((uint8_t)(6 + s_cnt), DEV_TYPE_SMOKE, 1, 0);
            s_cnt++;
            if (s_cnt >= 6U) {
                s_stage = S5_STORM; s_stage_tick = 0; s_storm_injected = 0;
                TestLog("S5 STORM begin (40x fire in one tick)");
            }
        }
        break;

    case S5_STORM: /* ����������ע40��(����32��, �ؼ���¼����������=���) */
        while (s_storm_injected < 40U) {
            StorageEvent_LogFire((uint8_t)(30 + s_storm_injected), DEV_TYPE_SMOKE, 1, 0);
            s_storm_injected++;
        }
        /* �ȶ����ſ��ٱ�ͳ��; 10s���� */
        if (s_stage_tick >= 100U) {
            TestLog("S5 end (expect 32��2x cmd=03, oldest dropped; M1: unplug PB6/PB7 NOW)");
            s_stage = S6_RESET; s_stage_tick = 0;
        }
        break;

    case S6_RESET: /* ��λ: ���׾���־+��¼��λ�¼� */
        if (s_stage_tick >= 50U) {
            StorageEvent_ResetFirstFire();
            StorageEvent_LogReset();
            TestLog("S6 RESET (EVT_RESET=122 zone3, first-fire flag cleared)");
            s_stage = S7_SECOND_FIRST; s_stage_tick = 0;
        }
        break;

    case S7_SECOND_FIRST: /* ��λ���ٻ�: Ӧ�ٴβ���0x02 */
        if (s_stage_tick >= 100U) {
            StorageEvent_LogFire(7, DEV_TYPE_TEMPERATURE, 1, 0);
            TestLog("S7 SECOND FIRST FIRE (cmd=02 AGAIN after reset)");
            s_stage = S8_SILENCE; s_stage_tick = 0;
            TestLog("S8 SILENCE 180s begin");
        }
        break;

    case S8_SILENCE: /* ��Ĭ3min: ��·����(���ߴ���������S5�ſ���, ��M1) */
        if (s_stage_tick >= 1800U) {
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

/* ---------- ��ʼ��(StorageTx_Init()β������) ---------- */
void StorageTest_Init(void)
{
    s_stage = S_IDLE; s_tick = 0; s_stage_tick = 0; s_cycle = 1;
    /* ������־���׸�Tick��(��ʱTaskLoop��ȷ�ϳ�ʼ�����) */
}
#endif /* STX_TEST_ENABLE */
