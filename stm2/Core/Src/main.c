/* STM2 USART1 TX DMA A/B pin-mode test + Next-Set Prescription (Instability Score).
 * NUCLEO-F103RB: replace Core/Src/main.c in the existing project.
 * Use the accompanying STM3/UNO files: summary fields are corrected together.
 * USART1 PA9/PA10 to STM1: 115200 8N1; TX DMA1 Channel4; RX IRQ.
 * USART3 PB10/PB11 to STM3: Polling TX, IRQ RX unchanged.
 * USART2 PA2 ST-LINK VCP: 115200 diagnostics + @BENCH workout reports.
 * Keypad D requests session completion via STM3; 32 set summaries in RAM.
 * TC gated; DMA complete != wire complete; no guaranteed fault-free link.
 * Prototype mock only; do not use for load-bearing safety systems.
 */
/* Shared wire format: A5 5A type id(LE32) a..g(LE32) CRC16-CCITT(LE16).
- 37 bytes; all peers must use this revision. */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#define FRAME_N 37u
#define T_RACK 1u
#define T_CONTROL 2u
#define T_MEASURE 3u
#define T_UI 4u
#define T_KEY 5u
#define F_ACTIVE 1u
#define F_ASSIST 2u
#define F_RACKED 4u
#define F_UNRACKED 8u
#define F_PARTIAL 16u
#define F_PARKED 32u
#define F_WEIGHT 64u
#define F_FSR_OK 128u
#define F_LINK 256u
#define F_SUMMARY 512u
#define F_WAIT_FINAL 1024u
#define F_SESSION_END (1u<<30) /* Rack request / UI acknowledgement. */
#define F_MODEL_ENABLED 2048u
#define M_CAL 1u
#define M_DISTANCE 2u
#define M_ACTIVE 4u
#define M_FINAL 8u
#define M_BASE 16u
#define M_TILT 32u
#define M_FAULT 64u
#define I_WEIGHT 1u
#define I_IMU 2u
#define I_MODEL 4u
#define I_LINK 8u
#define I_FSR 16u
#define I_FINAL 32u
#define I_DISTANCE 64u
#define I_ASSIST 128u
#define I_BASE 256u
#define REC_NONE 0u
#define REC_KEEP 1u
#define REC_REDUCE 2u
#define REC_INCREASE 3u
#define REASON_PEDAL 1u
#define REASON_FSR1 2u
#define U32(v) ((uint32_t)(v))

typedef struct {
    uint8_t type;
    uint32_t id,a,b,c,d,e,f,g;
} Packet;

typedef struct {
    uint8_t b[FRAME_N];
    unsigned n;
    uint32_t crc_errors,sync_drops;
} Parser;

static uint16_t crc16(const uint8_t *p,unsigned n) {
    uint16_t c=0xffff;
    while(n--) {
        c^=(uint16_t)*p++<<8;
        for(unsigned j=0;j<8;j++)
            c=(uint16_t)((c&0x8000u)?((uint32_t)c<<1)^0x1021u:(uint32_t)c<<1);
    }
    return c;
}
static void put32(uint8_t *p,uint32_t x) {
    for(unsigned j=0;j<4;j++)
        p[j]=(uint8_t)(x>>(8*j));
}
static uint32_t get32(const uint8_t *p) {
    return U32(p[0])|(U32(p[1])<<8)|(U32(p[2])<<16)|(U32(p[3])<<24);
}
static void encode(const Packet *p,uint8_t b[FRAME_N]) {
    b[0]=0xa5;
    b[1]=0x5a;
    b[2]=p->type;
    uint32_t w[8]={p->id,p->a,p->b,p->c,p->d,p->e,p->f,p->g};
    for(unsigned j=0;j<8;j++)
        put32(b+3+4*j,w[j]);
    uint16_t c=crc16(b+2,33);
    b[35]=(uint8_t)c;
    b[36]=(uint8_t)(c>>8);
}
static bool parse(Parser *s,uint8_t c,Packet *p) {
    if(!s->n) {
        if(c==0xa5)
            s->b[s->n++]=c;
        else
            s->sync_drops++;
        return false;
    }
    if(s->n==1&&c!=0x5a) {
        s->sync_drops++;
        s->n=(c==0xa5)?1:0;
        return false;
    }
    s->b[s->n++]=c;
    if(s->n<FRAME_N)
        return false;
    uint8_t *b=s->b;
    s->n=0;
    if(crc16(b+2,33)!=((uint16_t)b[35]|(uint16_t)b[36]<<8)) {
        s->crc_errors++;
        if(c==0xa5) {
            b[0]=c;
            s->n=1;
        }
        return false;
    }
    p->type=b[2];
    p->id=get32(b+3);
    p->a=get32(b+7);
    p->b=get32(b+11);
    p->c=get32(b+15);
    p->d=get32(b+19);
    p->e=get32(b+23);
    p->f=get32(b+27);
    p->g=get32(b+31);
    return true;
}

#define F_CONTACT 4096u /* both rack contacts raw pressed: suspend new rep completion */
#define F_M_READY 8192u
#define F_M_TILT 16384u
#define F_M_BASE 32768u
#define F_M_WARN 65536u
#define M_WARN 128u
#define M_REP_VALID 256u
/* V2 calibration transaction; all four boards must be updated together. */
#define T_CAL_STATUS 6u
#define F_CAL_BUSY (1u<<17)
#define F_CAL_FAIL (1u<<18)
#define F_CAL_REQUIRED (1u<<19)
#define CAL_NONE 0u
#define CAL_BUSY 1u
#define CAL_READY 2u
#define CAL_FAIL 3u
#define I_CAL 512u
/* Summary only: a[29:20] holds all ten invalid bits; g[31:16] is 0.1 kg.
 * Frame size, type IDs, CRC and all non-summary fields are unchanged. */
#define SUMMARY_INVALID_SHIFT 20u
#define SUMMARY_INVALID_MASK 0x3ffu

/* STM32F103RB, HSI 8 MHz, register-only. USART1/3 IRQ RX; USART2 debug TX. */
#define R(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define B(n) (1u<<(n))
#define RA 0x40010800u
#define RB 0x40010c00u
#define RC 0x40011000u
#define U1 0x40013800u
#define U3 0x40004800u
#define U2 0x40004400u
#define T4 0x40000800u
#define APB2 R(0x40021018u)
#define APB1 R(0x4002101cu)

static void pin(uint32_t port,unsigned n,unsigned cfg) {
    uint32_t a=port+(n>=8?4:0),s=(n%8)*4;
    R(a)=(R(a)&~(15u<<s))|(cfg<<s);
}
static void pin_high(uint32_t port,unsigned n) {
    R(port+0x10)=B(n);
}
static void base_init(void) {
    R(0x40021000)|=1;
    while(!(R(0x40021000)&2)) {
    }
    R(0x40021004)&=~3u;
    while(R(0x40021004)&12u) {
    }
    R(0x40021004)&=~((15u<<4)|(7u<<8)|(7u<<11));
    APB2|=B(0)|B(2)|B(3)|B(4)|B(14);
    APB1|=B(2)|B(17)|B(18);
    R(T4)=0;
    R(T4+0x28)=7999;
    R(T4+0x2c)=65535;
    R(T4+0x14)=1;
    R(T4+0x24)=0;
    R(T4)=1;
}
static uint32_t ms(void) {
    static uint16_t last;
    static uint32_t total;
    uint16_t cur=(uint16_t)R(T4+0x24);
    total+=(uint16_t)(cur-last);
    last=cur;
    return total;
}

#define RING_N 256u
typedef struct {
    volatile uint8_t q[RING_N],head,tail,err;
} Ring;
static Ring r1,r3;
static volatile uint32_t rx_bytes[2], rx_errors[2], rx_overflows[2];
static uint32_t good_frames[2], wrong_type[2], tx_frames[2], rack_frames, old_rack_frames;
static volatile uint32_t framing[2], noise[2], overrun[2];

static void irq_rx(uint32_t u,Ring *r) {
    uint32_t s=R(u);
    if(s&(B(5)|15u)) {
        uint8_t c=(uint8_t)R(u+4);
        uint8_t next=(uint8_t)(r->head+1);
        unsigned i=u==U1?0u:1u;
        if(s&B(5))
            rx_bytes[i]++;
        if(s&15u)
            rx_errors[i]++;
        if(s&B(1))
            framing[i]++;
        if(s&B(2))
            noise[i]++;
        if(s&B(3))
            overrun[i]++;
        if((s&B(5))&&next==r->tail)
            rx_overflows[i]++;
        if((s&15u)||next==r->tail)
            r->err=1;
        else if(s&B(5)) {
            r->q[r->head]=c;
            r->head=next;
        }
    }
}
void USART1_IRQHandler(void) {
    irq_rx(U1,&r1);
}
void USART3_IRQHandler(void) {
    irq_rx(U3,&r3);
}
static void uart_init(uint32_t u,unsigned baud) {
    if(u==U1) {
        /* Restore the proven Polling build GPIO mode (AF push-pull 50 MHz). */
        pin(RA,9,11);
        pin_high(RA,10);
        pin(RA,10,8);
    }
    else if(u==U3) {
        pin(RB,10,11);
        pin_high(RB,11);
        pin(RB,11,8);
    }
    else {
        pin(RA,2,11);
        pin(RA,3,4);
    }
    /* Explicit reset of UART settings, then 8N1, no flow control. */
    uint32_t rr=u==U1?0x4002100cu:0x40021010u;
    uint32_t mask=u==U1?B(14):u==U3?B(18):B(17);
    R(rr)|=mask;
    R(rr)&=~mask;
    R(u+0xc)=0;
    R(u+0x10)=0;
    R(u+0x14)=0;
    R(u+8)=(8000000u+baud/2)/baud;
    R(u+0xc)=B(13)|B(3)|B(2)|(u!=U2?B(5):0);
    if(u==U1)
        R(0xe000e104)=B(5);
    if(u==U3)
        R(0xe000e104)=B(7);
}
static bool take(Ring *r,Parser *s,Packet *p) {
    if(r->err) {
        r->err=0;
        s->n=0;
        r->tail=r->head;
    }
    while(r->tail!=r->head) {
        uint8_t c=r->q[r->tail];
        r->tail=(uint8_t)(r->tail+1);
        if(parse(s,c,p)) {
            good_frames[r==&r1?0:1]++;
            return true;
        }
    }
    return false;
}

/* ================================================================
 * STM2 USART1 TX DMA (DMA1_Channel4) - known-working Polling base.
 * ================================================================ */
#define U1_DMA_BASE     0x40020000u
#define U1_DMA_ISR      (U1_DMA_BASE + 0x00u)
#define U1_DMA_IFCR     (U1_DMA_BASE + 0x04u)
#define U1_DMA_CCR4     (U1_DMA_BASE + 0x44u)
#define U1_DMA_CNDTR4   (U1_DMA_BASE + 0x48u)
#define U1_DMA_CPAR4    (U1_DMA_BASE + 0x4Cu)
#define U1_DMA_CMAR4    (U1_DMA_BASE + 0x50u)
#define U1_DMA_ISR_TC4  B(13)
#define U1_DMA_ISR_TE4  B(15)
#define U1_DMA_IFCR_ALL4 (0xFu << 12)
#define U1_DMA_N        4u
#define U1_DMA_MAX_MS   30u

static uint8_t u1_frames[U1_DMA_N][FRAME_N];
static volatile uint8_t u1_q_head, u1_q_tail, u1_q_count;
static volatile uint8_t u1_dma_busy, u1_wait_tc;
static volatile uint32_t u1_queued, u1_started, u1_dma_completed;
static volatile uint32_t u1_wire_completed, u1_overflow, u1_dma_errors, u1_timeouts;
static uint32_t u1_started_at;

static uint32_t u1_irq_save(void) {
    uint32_t p;
    __asm__ volatile ("mrs %0, primask" : "=r" (p) :: "memory");
    __asm__ volatile ("cpsid i" ::: "memory");
    return p;
}
static void u1_irq_restore(uint32_t p) {
    __asm__ volatile ("msr primask, %0" :: "r" (p) : "memory");
}

static void u1_dma_init(void) {
    R(0x40021014u) |= B(0); /* RCC_AHBENR: DMA1EN */
    R(U1_DMA_CCR4) = 0u;
    R(U1_DMA_IFCR) = U1_DMA_IFCR_ALL4;
    R(U1_DMA_CPAR4) = U1 + 4u; /* USART1->DR */
    R(U1 + 0x14u) |= B(7); /* USART_CR3.DMAT */
    /* UART RX IRQs remain default highest priority. DMA lower. */
    (*(volatile uint8_t *)(uintptr_t)(0xe000e400u + 14u)) = 0x80u;
    R(0xe000e100u) = B(14); /* DMA1_Channel4_IRQn=14 */
}

static void u1_dma_service(uint32_t t) {
    uint32_t irq = u1_irq_save();
    if (u1_dma_busy && (uint32_t)(t - u1_started_at) > U1_DMA_MAX_MS) {
        R(U1_DMA_CCR4) &= ~B(0);
        R(U1_DMA_IFCR) = U1_DMA_IFCR_ALL4;
        u1_dma_busy = 0u;
        u1_timeouts++;
        if (u1_q_count) {
            u1_q_tail = (uint8_t)((u1_q_tail + 1u) % U1_DMA_N);
            u1_q_count--;
        }
        u1_wait_tc = 2u;
    }
    if (u1_wait_tc) {
        if (!(R(U1) & B(6))) { u1_irq_restore(irq); return; }
        if (u1_wait_tc == 1u) u1_wire_completed++;
        u1_wait_tc = 0u;
    }
    if (!u1_dma_busy && u1_q_count) {
        if (u1_started && !(R(U1) & B(6))) {
            u1_irq_restore(irq);
            return;
        }
        R(U1_DMA_CCR4) = 0u;
        R(U1_DMA_IFCR) = U1_DMA_IFCR_ALL4;
        R(U1_DMA_CNDTR4) = FRAME_N;
        R(U1_DMA_CPAR4) = U1 + 4u;
        R(U1_DMA_CMAR4) = (uint32_t)(uintptr_t)&u1_frames[u1_q_tail][0];
        u1_started_at = t;
        u1_dma_busy = 1u;
        u1_started++;
        R(U1_DMA_CCR4) = B(13) | B(7) | B(4) | B(3) | B(1) | B(0);
    }
    u1_irq_restore(irq);
}

void DMA1_Channel4_IRQHandler(void) {
    uint32_t status = R(U1_DMA_ISR);
    R(U1_DMA_IFCR) = U1_DMA_IFCR_ALL4;
    if (!(status & (U1_DMA_ISR_TC4 | U1_DMA_ISR_TE4))) return;
    R(U1_DMA_CCR4) &= ~B(0);
    if (!u1_dma_busy) return;
    if (status & U1_DMA_ISR_TE4) u1_dma_errors++;
    else if (status & U1_DMA_ISR_TC4) u1_dma_completed++;
    if (u1_q_count) {
        u1_q_tail = (uint8_t)((u1_q_tail + 1u) % U1_DMA_N);
        u1_q_count--;
    }
    u1_dma_busy = 0u;
    u1_wait_tc = (status & U1_DMA_ISR_TE4) ? 2u : 1u;
}

static bool u1_dma_enqueue(const uint8_t bytes[FRAME_N]) {
    uint32_t irq = u1_irq_save();
    if (u1_q_count == U1_DMA_N) {
        u1_overflow++;
        u1_irq_restore(irq);
        return false;
    }
    for (unsigned i = 0; i < FRAME_N; i++)
        u1_frames[u1_q_head][i] = bytes[i];
    u1_q_head = (uint8_t)((u1_q_head + 1u) % U1_DMA_N);
    u1_q_count++;
    u1_queued++;
    u1_irq_restore(irq);
    u1_dma_service(ms());
    return true;
}

static void sendp(uint32_t u, const Packet *p) {
    uint8_t b[FRAME_N];
    encode(p, b);
    if (u == U1) {
        if (u1_dma_enqueue(b)) tx_frames[0]++;
        return;
    }
    for (unsigned i = 0; i < FRAME_N; i++) {
        while (!(R(u) & B(7))) { }
        R(u + 4u) = b[i];
    }
    if (u == U3) tx_frames[1]++;
}

static char web_line[480];
static unsigned web_len,web_pos;
static void logstr(const char *s) {
    /* Keep diagnostic text out of a partially transmitted JSON line. */
    if(web_pos < web_len) return;
    while(*s) {
        while(!(R(U2)&B(7))) {
        }
        R(U2+4)=(uint8_t)*s++;
    }
}

/* ================================================================
 * 운동 불안정성(피로도) 점수 및 다음 세트 중량/횟수 추천 알고리즘
 * - VBT(Velocity Loss) 속도 저하율 + MPU6050 최대 기울기 + 보조 개입 기반
 * ================================================================ */
typedef struct {
    uint32_t next_weight;   /* 추천 다음 세트 중량 (0.1 kg) */
    uint32_t next_reps;     /* 추천 다음 세트 목표 횟수 (회) */
    uint32_t instab_score;  /* 종합 불안정성 점수 (0 ~ 100) */
    uint32_t vl_pct;        /* 속도 저하율 (%) */
    uint8_t  rec_code;      /* REC_NONE / REC_KEEP / REC_REDUCE / REC_INCREASE */
    uint8_t  grade;         /* 1:위험감량, 2:피로감량, 3:유지, 4:증량 */
} SetPrescription_t;

static SetPrescription_t last_rx;

/* Keep 0.1 kg units throughout, including half-kilogram plate increments. */
static uint32_t round_barbell_x10(uint32_t kg_x10) {
    uint32_t rounded = ((kg_x10 + 12u) / 25u) * 25u;
    return (rounded < 200u) ? 200u : rounded; /* Existing 20 kg minimum. */
}

static SetPrescription_t calc_next_set(uint32_t w_x10, uint32_t cur_reps,
                                       uint32_t first_ms, uint32_t last_ms,
                                       uint32_t tilt_tenth, bool is_assisted,
                                       uint32_t inv_flags) {
    SetPrescription_t res = { w_x10, cur_reps, 0u, 0u, REC_NONE, 0u };

    if (w_x10 == 0u || w_x10 > 9999u) return res;

    /* 보조 개입(깔림)이 발생했거나, 3회 이상 정상 측정된 경우에만 판정 수행 */
    if (!is_assisted && (inv_flags != 0u || cur_reps < 3u || first_ms == 0u)) {
        return res;
    }

    /* 1) 속도 저하율(VL %) 계산: 초반 상승 시간(1~2회 평균) 대비 마지막 반복 상승 시간 증가율 */
    uint32_t first_avg_ms = (cur_reps >= 2u) ? (first_ms / 2u) : first_ms;
    uint32_t vl = 0u;
    if (first_avg_ms > 0u && last_ms > first_avg_ms) {
        uint64_t scaled = ((uint64_t)(last_ms - first_avg_ms) * 100u) / first_avg_ms;
        vl = scaled > UINT32_MAX ? UINT32_MAX : (uint32_t)scaled;
    }
    res.vl_pct = vl;

    /* 2) 종합 불안정성 지수 (0 ~ 100점)
     *    - 속도 저하 비중(60%): VL(%) * 1.2
     *    - 자세 흔들림 비중(40%): 최대 기울기(도) * 5.0 (= tilt_tenth / 2) */
    uint64_t score = ((uint64_t)vl * 12u) / 10u + (tilt_tenth / 2u);
    if (is_assisted && score < 80u) score = 80u; /* 보조 개입 시 최소 80점 부여 */
    if (score > 100u) score = 100u;
    res.instab_score = (uint32_t)score;

    /* 3) 실제 트레이닝 처방 로직 (4단계) */
    if (is_assisted || tilt_tenth >= 70u || score >= 70u) {
        /* [1등급: 위험 / 깔림 발생] 15% 감량, 횟수 유지(최소 5회) */
        res.grade       = 1u;
        res.rec_code    = REC_REDUCE;
        res.next_weight = round_barbell_x10(w_x10 * 85u / 100u);
        res.next_reps   = (cur_reps < 5u) ? 5u : cur_reps;
    } else if (vl >= 30u || tilt_tenth >= 50u || score >= 45u) {
        /* [2등급: 피로 누적 / 불안정] 10% 감량, 횟수 유지 */
        res.grade       = 2u;
        res.rec_code    = REC_REDUCE;
        res.next_weight = round_barbell_x10(w_x10 * 90u / 100u);
        res.next_reps   = cur_reps;
    } else if (vl < 15u && tilt_tenth < 30u && cur_reps >= 8u) {
        /* [4등급: 매우 안정 / 여유 있음] +2.5kg 증량, 횟수 2회 감소 (최소 6회) */
        res.grade       = 4u;
        res.rec_code    = REC_INCREASE;
        res.next_weight = round_barbell_x10(w_x10 + 25u);
        res.next_reps   = (cur_reps > 6u) ? (cur_reps - 2u) : 6u;
    } else {
        /* [3등급: 적정 자극] 중량 유지, 누적 피로 반영해 1회 감소 (최소 5회) */
        res.grade       = 3u;
        res.rec_code    = REC_KEEP;
        res.next_weight = w_x10;
        res.next_reps   = (cur_reps > 5u) ? (cur_reps - 1u) : cur_reps;
    }

    /* Suppress advice that cannot be represented or re-entered on the keypad. */
    if (res.next_weight > 9999u || res.next_reps > 255u) res.rec_code = REC_NONE;
    return res;
}

/* STM2: coordinator, cumulative measurements (no duplicate reps), last summary. */
static Parser mparser,rparser;
static Packet rack,measurement,final_summary;
static uint32_t id,weight,reps,total,first,last,max_tilt,invalid,cause,measure_at,rack_at,send_at,log_at,end_at;
typedef struct {
    uint32_t up_ms,peak_tenth,drop_mm;
    bool valid;
} RepRecord;
#define REP_LOG_N 256u
static RepRecord records[REP_LOG_N];
static uint32_t mf,cal_at;
static Packet cal;
static bool seen_cal;
static bool active_set,assisted,waiting,summary,seen_rack,seen_measure;

/* Retain set summaries on STM2; the laptop only presents these results.
 * The last completed workout remains available until another D completion.
 * A bounded log reports overflow explicitly instead of silently dropping sets. */
#define SESSION_SET_N 32u
typedef struct {
    uint32_t id,weight,reps,avg,last,tilt,invalid,cause,assisted;
    SetPrescription_t advice;
} SetResult;
static SetResult session_sets[SESSION_SET_N],web_sets[SESSION_SET_N];
static uint32_t session_count,session_dropped,stored_id;
static uint32_t web_count,web_dropped,web_session,web_cycle,web_at;
static unsigned web_index;
static bool session_ended,end_requested,web_sending;

static void store_set(void) {
    if(!id || stored_id==id) return;
    stored_id=id;
    if(session_count==SESSION_SET_N) { session_dropped++; return; }
    session_sets[session_count++]=(SetResult){id,weight,reps,
        reps?total/reps:0,last,max_tilt,invalid,cause,assisted?1u:0u,last_rx};
}
static void complete_session(uint32_t t) {
    if(!end_requested || session_ended || active_set || waiting) return;
    session_ended=true;
    end_requested=false;
    memcpy(web_sets,session_sets,session_count*sizeof(SetResult));
    web_count=session_count;
    web_dropped=session_dropped;
    if(++web_session==0) ++web_session;
    /* Finish any old line, then restart with a new transaction header. */
    web_sending=false;
    web_at=t-3000u;
}
static void web_service(uint32_t t) {
    if(web_pos<web_len) {
        if(R(U2)&B(7)) R(U2+4)=(uint8_t)web_line[web_pos++];
        return;
    }
    if(!web_session) return;
    if(!web_sending) {
        if(t-web_at<3000u) return;
        web_sending=true;
        web_index=0;
        if(++web_cycle==0) ++web_cycle;
    }
    int n;
    if(web_index==0) {
        n=snprintf(web_line,sizeof web_line,
            "\n@BENCH {\"v\":1,\"type\":\"begin\",\"session\":%lu,\"cycle\":%lu,\"count\":%lu,\"dropped\":%lu}\r\n",
            (unsigned long)web_session,(unsigned long)web_cycle,
            (unsigned long)web_count,(unsigned long)web_dropped);
    } else if(web_index<=web_count) {
        const SetResult *s=&web_sets[web_index-1];
        n=snprintf(web_line,sizeof web_line,
            "\n@BENCH {\"v\":1,\"type\":\"set\",\"session\":%lu,\"cycle\":%lu,\"index\":%u,\"id\":%lu,\"weight\":%lu,\"reps\":%lu,\"avg\":%lu,\"last\":%lu,\"tilt\":%lu,\"invalid\":%lu,\"cause\":%lu,\"assisted\":%lu,\"score\":%lu,\"timeIncrease\":%lu,\"rec\":%u,\"nextWeight\":%lu,\"nextReps\":%lu}\r\n",
            (unsigned long)web_session,(unsigned long)web_cycle,web_index,
            (unsigned long)s->id,(unsigned long)s->weight,(unsigned long)s->reps,
            (unsigned long)s->avg,(unsigned long)s->last,(unsigned long)s->tilt,
            (unsigned long)s->invalid,(unsigned long)s->cause,(unsigned long)s->assisted,
            (unsigned long)s->advice.instab_score,(unsigned long)s->advice.vl_pct,
            (unsigned)s->advice.rec_code,(unsigned long)s->advice.next_weight,
            (unsigned long)s->advice.next_reps);
    } else {
        n=snprintf(web_line,sizeof web_line,
            "\n@BENCH {\"v\":1,\"type\":\"end\",\"session\":%lu,\"cycle\":%lu,\"count\":%lu}\r\n",
            (unsigned long)web_session,(unsigned long)web_cycle,(unsigned long)web_count);
        web_sending=false;
        web_at=t;
    }
    web_index++;
    web_pos=0;
    web_len=(n>0 && (unsigned)n<sizeof web_line)?(unsigned)n:0;
}

static void begin(uint32_t n,uint32_t w,uint32_t bad) {
    if(session_ended) {
        session_count=session_dropped=0;
        session_ended=false;
    }
    end_requested=false;
    id=n;
    weight=w;
    memset(records,0,sizeof records);
    reps=total=first=last=max_tilt=cause=0;
    invalid=bad;
    active_set=true;
    assisted=waiting=summary=false;
    mf=0;
    memset(&last_rx, 0, sizeof last_rx);
}
static bool calibration_ready(uint32_t token,uint32_t t) {
    return token&&seen_cal&&t-cal_at<=500u&&cal.a==token&&cal.b==CAL_READY&&seen_measure&&t-measure_at<=500u&&((measurement.a&(M_CAL|M_DISTANCE))==(M_CAL|M_DISTANCE));
}
static bool rack_fresh(uint32_t t) {
    return seen_rack&&t-rack_at<=1000u;
}
static bool fsr_displayable(uint32_t t) {
    return rack_fresh(t)&&(rack.e&0xffffu)<=4095u&&(rack.e>>16)<=4095u&&(rack.f&0xffffu)<=4095u;
}
static uint32_t cal_ui_flags(uint32_t t) {
    if(!rack_fresh(t)||!rack.g)
        return F_CAL_REQUIRED;
    if(calibration_ready(rack.g,t))
        return F_M_READY;
    if(seen_cal&&t-cal_at<=500u&&cal.a==rack.g&&cal.b==CAL_FAIL)
        return F_CAL_FAIL;
    return F_CAL_BUSY;
}
static Packet make_ui(void) {
    uint32_t flags=rack.a & ~F_SESSION_END;
    if(session_ended) flags|=F_SESSION_END;
    if(summary)
        flags|=F_SUMMARY;
    if(waiting)
        flags|=F_WAIT_FINAL;
    if(!active_set)
        flags|=cal_ui_flags(ms());
    else if(!(invalid&I_CAL)&&(mf&M_CAL)&&(mf&M_DISTANCE))
        flags|=F_M_READY;
    if(mf&M_TILT)
        flags|=F_M_TILT;
    if(mf&M_BASE)
        flags|=F_M_BASE;
    if(mf&M_WARN)
        flags|=F_M_WARN;

    /* T_UI summary: a[29:20]=invalid, f[15:0]=tilt, f[23:16]=score,
     * f[31:24]=next reps, g[7:0]=recommendation, g[15:8]=cause,
     * g[31:16]=exact next weight in 0.1 kg. REC_NONE has no advice payload.
     * Non-summary: f[31:16]=cal token, g[31:16]=invalid as before. */
    uint32_t f_word = (max_tilt & 0xffffu);
    uint32_t g_word = (cause & 0xffu) << 8;

    if (summary) {
        flags |= (invalid & SUMMARY_INVALID_MASK) << SUMMARY_INVALID_SHIFT;
        if (last_rx.rec_code != REC_NONE) {
            f_word |= ((last_rx.instab_score & 0xffu) << 16) | ((last_rx.next_reps & 0xffu) << 24);
            g_word |= (last_rx.rec_code & 0xffu) | ((last_rx.next_weight & 0xffffu) << 16);
        }
    } else {
        f_word |= ((rack.g & 0xffffu) << 16);
        g_word |= ((invalid & 0xffffu) << 16);
    }

    Packet p={T_UI,id,flags,weight,reps,reps?total/reps:0,last,f_word,g_word};
    return p;
}
static void finish(void) {
    waiting=false;
    /* An empty, otherwise valid set returns directly to the next-set screen. */
    summary=(reps!=0u || invalid!=0u || assisted);
    last_rx = calc_next_set(weight, reps, first, last, max_tilt, assisted, invalid);
    final_summary=make_ui();
    store_set();

    char b[220];
    snprintf(b,
             sizeof b,
             "[결과] 세트 %lu | %lu회 | 평균 %lu ms | 마지막 %lu ms | 오류 0x%lx\r\n",
             (unsigned long)id,
             (unsigned long)reps,
             (unsigned long)final_summary.d,
             (unsigned long)last,
             (unsigned long)invalid);
    logstr(b);

    if (last_rx.rec_code != REC_NONE) {
        const char *g_str = (last_rx.grade == 1u) ? "위험감량" :
                            (last_rx.grade == 2u) ? "피로감량" :
                            (last_rx.grade == 4u) ? "안정증량" : "적정유지";
        snprintf(b,
                 sizeof b,
                 "[AI 코칭] 불안정성 %lu점 (속도저하 %lu%%, 기울기 %lu.%lu도) -> [%s] 다음 세트: %lu.%lu kg x %lu회\r\n",
                 (unsigned long)last_rx.instab_score,
                 (unsigned long)last_rx.vl_pct,
                 (unsigned long)(max_tilt / 10u),
                 (unsigned long)(max_tilt % 10u),
                 g_str,
                 (unsigned long)(last_rx.next_weight / 10u),
                 (unsigned long)(last_rx.next_weight % 10u),
                 (unsigned long)last_rx.next_reps);
        logstr(b);
    } else {
        logstr("[AI 코칭] 유효 반복 부족(3회 미만) 또는 센서 미보정으로 추천 보류\r\n");
    }
}
static void on_rack(Packet p,uint32_t t) {
    if(p.type!=T_RACK) {
        wrong_type[1]++;
        return;
    }
    if(p.id<id) {
        old_rack_frames++;
        return;
    }
    seen_rack=true;
    rack_at=t;
    rack=p;
    rack_frames++;
    if((p.a&F_ACTIVE)&&p.id>id) {
        if(active_set || waiting) {
            invalid|=I_FINAL;
            active_set=false;
            finish();
        }
        complete_session(t);
        uint32_t bad=p.d;
        if(!calibration_ready(p.g,t))
            bad|=I_CAL;
        if(!seen_measure||t-measure_at>1000||!(measurement.a&M_CAL))
            bad|=I_IMU;
        if(!seen_measure||t-measure_at>1000||!(measurement.a&M_DISTANCE))
            bad|=I_DISTANCE;
        begin(p.id,p.b,bad);
        logstr("[STM2] 새 세트\r\n");
    }
    if(p.id==id&&id&&(active_set||waiting)) {
        invalid|=p.d;
        if((p.a&F_ASSIST)&&!assisted) {
            assisted=true;
            cause=p.c;
            invalid|=I_ASSIST;
            logstr("[보조] 반복 기록 동결\r\n");
        }
        if(active_set&&!(p.a&F_ACTIVE)) {
            active_set=false;
            waiting=true;
            end_at=t;
            if(assisted)
                finish();
            else
                logstr("[재거치] STM1 최종 기록 대기\r\n");
        }
    }
    if(p.id==id && id && !(p.a&F_ACTIVE) &&
       (p.a&(F_RACKED|F_CONTACT|F_SESSION_END))==(F_RACKED|F_CONTACT|F_SESSION_END))
        end_requested=true;
    complete_session(t);
}
static void on_measure(Packet p,uint32_t t) {
    if(p.type==T_CAL_STATUS) {
        seen_cal=true;
        cal_at=t;
        cal=p;
        return;
    }
    if(p.type!=T_MEASURE) {
        wrong_type[0]++;
        return;
    }
    seen_measure=true;
    measure_at=t;
    measurement=p;
    if(!id) {
        mf=p.a;
        return;
    }
    if(p.id!=id)
        return;
    if(!(active_set||waiting)||assisted)
        return;
    /* Keep the completed set's tilt/base flags while the next set calibrates. */
    mf=p.a;
    if(p.a&M_FAULT)
        invalid|=I_DISTANCE;
    if(!(p.a&M_CAL))
        invalid|=I_IMU;
    if(!(invalid&I_CAL)&&p.b>=reps&&p.b<=65535&&p.c>=total) {
        if(p.b>reps&&p.b<=REP_LOG_N) {
            records[p.b-1]=(RepRecord){p.e,p.g>>16,p.g&0xffffu,(p.a&M_REP_VALID)!=0};
            if(p.b>reps+1)
                invalid|=I_LINK;
        }
        if(p.b>REP_LOG_N)
            invalid|=I_FINAL;
        reps=p.b;
        total=p.c;
        first=p.d;
        last=p.e;
        max_tilt=p.f;
    }
    if(waiting&&(p.a&M_FINAL)) {
        if(!(p.a&M_BASE))
            invalid|=I_BASE;
        finish();
    }
}
static void cal_debug(uint32_t t) {
    bool r_ok=rack_fresh(t);
    bool m_ok=seen_measure&&t-measure_at<=500u;
    bool c_ok=seen_cal&&t-cal_at<=500u;
    uint32_t ui_flags=cal_ui_flags(t);
    const char *cs=!c_ok?"NO_STATUS":cal.b==CAL_NONE?"NONE":cal.b==CAL_BUSY?"BUSY":cal.b==CAL_READY?"READY":cal.b==CAL_FAIL?"FAIL":"UNKNOWN";
    const char *us=ui_flags==F_M_READY?"READY":ui_flags==F_CAL_FAIL?"FAIL":ui_flags==F_CAL_REQUIRED?"REQUIRED":"BUSY";
    char b[340];
    snprintf(b,
             sizeof b,
             "[CALDBG] STM3=%s STM1=%s CAL=%s\r\nREQ=%lu RX_TOKEN=%lu RACK=%u CONTACT=%u ACTIVE=%u\r\nMPU=%u TOF=%u HEIGHT=%lu UI=%s\r\n",
             r_ok?"OK":"LOST",
             m_ok?"OK":"LOST",
             cs,
             (unsigned long)rack.g,
             (unsigned long)cal.a,
             (unsigned)((rack.a&F_RACKED)!=0),
             (unsigned)((rack.a&F_CONTACT)!=0),
             (unsigned)((rack.a&F_ACTIVE)!=0),
             (unsigned)(m_ok&&(measurement.a&M_CAL)!=0),
             (unsigned)(m_ok&&(measurement.a&M_DISTANCE)!=0),
             (unsigned long)cal.c,
             us);
    logstr(b);
    snprintf(b,
             sizeof b,
             "[UART1 STM1] RX=%lu ERR=%lu OVF=%lu PKT=%lu\r\n[UART3 STM3] RX=%lu ERR=%lu OVF=%lu PKT=%lu RACKPKT=%lu\r\n",
             (unsigned long)rx_bytes[0],
             (unsigned long)rx_errors[0],
             (unsigned long)rx_overflows[0],
             (unsigned long)good_frames[0],
             (unsigned long)rx_bytes[1],
             (unsigned long)rx_errors[1],
             (unsigned long)rx_overflows[1],
             (unsigned long)good_frames[1],
             (unsigned long)rack_frames);
    logstr(b);
    snprintf(b,
             sizeof b,
             "[U1] TX=%lu CRC=%lu TYPE=%lu FE=%lu NE=%lu ORE=%lu PIN=%u\r\n[U3] TX=%lu CRC=%lu TYPE=%lu FE=%lu NE=%lu ORE=%lu PIN=%u OLD_ID=%lu\r\n",
             (unsigned long)tx_frames[0],
             (unsigned long)mparser.crc_errors,
             (unsigned long)wrong_type[0],
             (unsigned long)framing[0],
             (unsigned long)noise[0],
             (unsigned long)overrun[0],
             (unsigned)((R(RA+8)&B(10))!=0),
             (unsigned long)tx_frames[1],
             (unsigned long)rparser.crc_errors,
             (unsigned long)wrong_type[1],
             (unsigned long)framing[1],
             (unsigned long)noise[1],
             (unsigned long)overrun[1],
             (unsigned)((R(RB+8)&B(11))!=0),
             (unsigned long)old_rack_frames);
    logstr(b);
    snprintf(b,
             sizeof b,
             "[CTRL] SEND=%s REQ=%lu | RACK_AGE=%lu ms | REASON=%s\r\n",
             r_ok?"ON":"OFF",
             (unsigned long)(r_ok&&!waiting?rack.g:0),
             (unsigned long)(seen_rack?t-rack_at:0),
             !r_ok?"NO_FRESH_RACK":!rack.g?"NO_REQUEST":waiting?"WAIT_FINAL":(rack.a&F_ACTIVE)?"ACTIVE":!(rack.a&F_RACKED)?"NOT_RACKED":!(rack.a&F_CONTACT)?"NO_CONTACT":!m_ok?"NO_STM1_MEASURE":!c_ok?"NO_CAL_STATUS":cal.a!=rack.g?"TOKEN_MISMATCH":cal.b==CAL_FAIL?"CAL_FAILED":cal.b==CAL_BUSY?"CAL_BUSY":ui_flags==F_M_READY?"READY":"CHECK_FLAGS");
    logstr(b);
    unsigned x=(unsigned)(rack.e&0xffffu),y=(unsigned)(rack.e>>16),z=(unsigned)(rack.f&0xffffu);
    if(!r_ok) {
        snprintf(b,sizeof b,"[FSR] N/A | %s\r\n",seen_rack?"STM3 LOST (old sample)":"NO PACKET");
    }
    else if(!fsr_displayable(t)) {
        snprintf(b,sizeof b,"[FSR] INVALID RANGE | %u,%u,%u\r\n",x,y,z);
    }
    else {
        snprintf(b,
                 sizeof b,
                 "[FSR RAW] %u,%u,%u | packet age=%lu ms | FSR_OK=%u (raw, not proof of calibration)\r\n",
                 x,
                 y,
                 z,
                 (unsigned long)(t-rack_at),
                 (unsigned)((rack.a&F_FSR_OK)!=0));
    }
    logstr(b);
    snprintf(b, sizeof b,
             "[U1 DMA TX] q=%lu start=%lu dma_done=%lu wire_done=%lu pending=%u busy=%u drops=%lu te=%lu timeout=%lu TC=%u\r\n",
             (unsigned long)u1_queued, (unsigned long)u1_started,
             (unsigned long)u1_dma_completed, (unsigned long)u1_wire_completed,
             (unsigned)u1_q_count, (unsigned)u1_dma_busy,
             (unsigned long)u1_overflow, (unsigned long)u1_dma_errors,
             (unsigned long)u1_timeouts, (unsigned)((R(U1) & B(6))!=0));
    logstr(b);
}
static void tick(uint32_t t) {
    if(active_set||waiting) {
        if(!seen_measure||t-measure_at>1000||!seen_rack||t-rack_at>1000)
            invalid|=I_LINK;
    }
    if(waiting&&t-end_at>=1500) {
        invalid|=I_FINAL;
        finish();
    }
    complete_session(t);
    if(t-send_at>=100) {
        send_at=t;
        uint32_t f=rack.a;
        if(!seen_rack||t-rack_at>1000)
            f&=~(F_ACTIVE|F_RACKED);
        if(active_set&&(invalid&I_CAL))
            f|=F_CAL_FAIL;
        Packet ctrl={T_CONTROL,id,f,rack.b,waiting?0:rack.g,0,0,0,0};
        if(rack_fresh(t))
            sendp(U1,&ctrl);
        Packet ui=make_ui();
        if(rack_fresh(t)) {
            sendp(U3,&ui);
            /* Summary uses f for advice: forward the existing token-bearing
             * calibration packet so STM3 can still validate READY. */
            if(seen_cal && t-cal_at<=500u && cal.a==rack.g)
                sendp(U3,&cal);
        }
    }
    if(t-log_at>=1000) {
        log_at=t;
        char b[130];
        snprintf(b,
                 sizeof b,
                 "[STM2] ID %lu | %lu회 | active %u | assist %u | invalid %lx\r\n",
                 (unsigned long)id,
                 (unsigned long)reps,
                 active_set,
                 assisted,
                 (unsigned long)invalid);
        logstr(b);
        cal_debug(t);
    }
}
int main(void) {
    base_init();
    uart_init(U1,115200);
    u1_dma_init();
    uart_init(U3,115200);
    uart_init(U2,115200);
    logstr("\r\n[STM2] PRJ2 DEBUG V3 + USART1 TX DMA CH4 + AI Set Prescription\r\n[STM2] 115200 8N1 / 37B V2 packet / no local ADC\r\n");
    for(;;) {
        uint32_t t=ms();
        Packet p;
        while(take(&r3,&rparser,&p))
            on_rack(p,t);
        while(take(&r1,&mparser,&p))
            on_measure(p,t);
        u1_dma_service(ms());
        tick(ms());
        u1_dma_service(ms());
        web_service(ms());
    }
}

