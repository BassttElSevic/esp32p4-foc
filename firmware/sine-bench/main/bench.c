/* Single-shot M2 bench. Only the motor task may drive the PWM. The console
 * and the independent timer may always remove EN, even during calibration. */
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_console.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "bench_io.h"
#include "foc_parameters.h"

/* Commissioning envelope, intentionally separate from the motor PI tuning.
 * Requires an R015 bipolar M2 board, nominal 12 V supply and a free bare shaft. */
#define MAX_PHASE_MA        220.0f
#define IQ_LIMIT_MA          60.0f
#define MAX_REF_SPEED         5.0f     /* electrical rad/s */
#define MAX_RAW_SPEED        60.0f     /* electrical rad/s, including encoder quantization */
#define MAX_FILTERED_SPEED   35.0f
#define MOD_LIMIT             0.12f
#define ALIGN_MOD             0.10f
#define MIN_FREQ_HZ           0.5f
#define MAX_FREQ_HZ          10.0f
#define MAX_SECONDS           8
#define LEASE_US         500000
#define LOOP_GAP_US       10000
#define RECORD_US          4000
#define MAX_RECORDS (1 + MAX_SECONDS * 1000000 / RECORD_US)
#define STUCK_CHECK_MOD      0.020f
#define STUCK_CHECK_US      120000

typedef enum { BOOT, UNCAL, IDLE, CAL, TEST, DONE, FAULT, DUMP } state_t;
typedef enum { ALIGN, CURRENT, SPEED } bench_mode_t;
static const char *const states[] = {"BOOT","UNCAL","IDLE","CAL","TEST","DONE","FAULT","DUMP"};
static const char *const modes[] = {"cal","current","speed"};
typedef struct { bench_mode_t mode; float hz, amp; int cycles; } request_t;
typedef struct { int t_us, code; float ref, iq, iqf, raw, speed; bool limited; } sample_t;
typedef struct { float off_u, off_v, angle_zero, direction; } calibration_t;
typedef struct { float u, v, w, angle; int code; int64_t time; } sensors_t;

static sample_t record[MAX_RECORDS];
static size_t count;
static calibration_t cal;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static state_t state = BOOT;
static request_t request;
static const char *abort_reason;
static int64_t deadline, lease, last_tick;
static bool enabled;
static TaskHandle_t motor;

static bool active(void) { return state == CAL || state == TEST; } /* under lock */

/* Caller holds lock. EN is removed before logging, neutral PWM or any wait. */
static void trip(const char *reason)
{
    bench_io_disable();
    enabled = false;
    if (active() && (!abort_reason || strcmp(abort_reason,"ok")==0)) abort_reason = reason;
}

static void check_time(int64_t now)
{
    if (!active() || abort_reason) return;
    if (now >= lease) trip("link_timeout");
    else if (enabled && now-last_tick >= LOOP_GAP_US) trip("control_timeout");
    else if (now >= deadline) trip(state == TEST && enabled ? "ok" : "deadline");
}

static void watchdog(void *arg)
{
    (void)arg;
    portENTER_CRITICAL(&lock);
    check_time(esp_timer_get_time());
    portEXIT_CRITICAL(&lock);
}

static bool live(void)
{
    portENTER_CRITICAL(&lock);
    check_time(esp_timer_get_time());
    bool ok = active() && !abort_reason;
    portEXIT_CRITICAL(&lock);
    return ok;
}

static bool fail(const char *reason)
{
    portENTER_CRITICAL(&lock);
    trip(reason);
    portEXIT_CRITICAL(&lock);
    return false;
}

static bool arm(int64_t now, int64_t run_us)
{
    if (!bench_io_voltage(0,0,0)) return fail("pwm");
    portENTER_CRITICAL(&lock);
    int64_t time = esp_timer_get_time();
    check_time(time);
    bool ok = active() && !abort_reason && time-now < LOOP_GAP_US;
    if (ok) {
        if (state == TEST) deadline = now + run_us;
        last_tick = now;
        enabled = true;
        bench_io_enable();
    } else if (!abort_reason) trip("stale_sample");
    portEXIT_CRITICAL(&lock);
    return ok;
}

static bool apply(float theta, float d, float q, int64_t sample_time)
{
    portENTER_CRITICAL(&lock);
    int64_t time = esp_timer_get_time();
    check_time(time);
    if (time-sample_time >= LOOP_GAP_US) trip("stale_sample");
    bool ok = enabled && !abort_reason;
    if (ok) last_tick = sample_time;
    portEXIT_CRITICAL(&lock);
    if (!ok) return false;
    /* stop/watchdog can still remove EN here. Writing PWM never re-enables it. */
    return bench_io_voltage(theta,d,q) || fail("pwm");
}

static bool read_sensors(sensors_t *s)
{
    float u,v;
    if (!live()) return false;
    if (!bench_io_sample(&u,&v,&s->code)) return fail("adc_or_encoder");
    s->time = esp_timer_get_time();
    if (!isfinite(u) || !isfinite(v) || u < RAIL_MARGIN_MV || v < RAIL_MARGIN_MV ||
        u > ADC_FULL_MV-RAIL_MARGIN_MV || v > ADC_FULL_MV-RAIL_MARGIN_MV)
        return fail("adc_rail");
    const float scale = SENSE_SIGN/(AMP_GAIN*R_SHUNT_OHM);
    s->u = (u-cal.off_u)*scale;
    s->v = (v-cal.off_v)*scale;
    s->w = -s->u-s->v;
    s->angle = s->code*(TWO_PI_F/AS5600_RES);
    if (fmaxf(fabsf(s->u),fmaxf(fabsf(s->v),fabsf(s->w))) >= MAX_PHASE_MA)
        return fail("phase_overcurrent");
    return live();
}

static float angle_delta(float a,float b) { return remainderf(a-b,TWO_PI_F); }

/* Check that each independent INA output really sits near the bipolar
 * reference. Observe shaft motion over a window with the drive disabled. */
static bool preflight(bool calibrating, sensors_t *last)
{
    float sum_u=0,sum_v=0, lo_u=INFINITY, hi_u=-INFINITY, lo_v=INFINITY, hi_v=-INFINITY;
    int first=0, code=0;
    for (int n=0;n<64;n++) {
        float u,v;
        if (!live()) return false;
        if (!bench_io_sample(&u,&v,&code)) return fail("adc_or_encoder");
        if (!isfinite(u) || !isfinite(v) || u<1200 || u>2100 || v<1200 || v>2100)
            return fail("bipolar_zero");
        if(n==0) first=code;
        if (fabsf(angle_delta(code*TWO_PI_F/AS5600_RES, first*TWO_PI_F/AS5600_RES)) >
            3*TWO_PI_F/AS5600_RES) return fail("shaft_moving");
        lo_u=fminf(lo_u,u); hi_u=fmaxf(hi_u,u);
        lo_v=fminf(lo_v,v); hi_v=fmaxf(hi_v,v);
        sum_u+=u; sum_v+=v;
        vTaskDelay(1);
    }
    if (hi_u-lo_u>15 || hi_v-lo_v>15) return fail("noisy_zero");
    if (!calibrating && (fabsf(sum_u/64-cal.off_u)>15 || fabsf(sum_v/64-cal.off_v)>15))
        return fail("zero_shift");
    cal.off_u=sum_u/64; cal.off_v=sum_v/64;
    return read_sensors(last);
}

static bool speed_sample(const sensors_t *s,const sensors_t *previous,float *dt,float *raw)
{
    int64_t elapsed = s->time-previous->time;
    if (elapsed<100 || elapsed>=LOOP_GAP_US) {
        fail("sample_interval");
        return false;
    }
    *dt=elapsed*1e-6f;
    *raw = angle_delta(s->angle,previous->angle)*MOTOR_POLE_PAIRS_INIT/(*dt);
    if (fabsf(*raw)>MAX_RAW_SPEED) return fail("overspeed");
    return true;
}

static bool align_axis(void)
{
    sensors_t s;
    if (!preflight(true,&s) || !arm(s.time,0)) return false;
    float aligned[2];
    for(int pos=0;pos<2;pos++) {
        int64_t t0=s.time;
        float last_angle=s.angle;
        bool checked_current=false;
        do {
            sensors_t previous=s;
            vTaskDelay(1);
            if (!read_sensors(&s)) return false;
            float dt,raw;
            if (!speed_sample(&s,&previous,&dt,&raw)) return false;
            float t=(s.time-t0)*1e-6f;
            float ramp=fminf(1,t/0.35f);
            float theta=pos*ramp*TWO_PI_F/4;
            if (!apply(theta,ALIGN_MOD*(pos ? 1:ramp),0,s.time)) return false;
            if (t<1.15f) last_angle=s.angle;
            if (t>1.15f) {
                if (fabsf(angle_delta(s.angle,last_angle))>3*TWO_PI_F/AS5600_RES)
                    return fail("alignment_moving");
                /* Stationary alpha/beta currents must follow the commanded
                 * stator field, including sign; a flat or reversed shunt fails. */
                float beta=(s.u+2*s.v)*ONE_OVER_SQRT3;
                float direct=pos ? beta:s.u;
                float cross=pos ? s.u:beta;
                if (direct < 15 || fabsf(cross)>0.45f*direct+5)
                    return fail("current_polarity");
                checked_current=true;
            }
        } while(s.time-t0 < 1350000);
        if (!checked_current) return fail("alignment_timing");
        aligned[pos]=s.angle;
    }
    float delta=angle_delta(aligned[1],aligned[0]);
    float expected=TWO_PI_F/(4*MOTOR_POLE_PAIRS_INIT);
    if (fabsf(fabsf(delta)-expected)>0.15f*expected) return fail("angle_travel");
    cal.direction=delta>0 ? 1:-1;
    cal.angle_zero=-cal.direction*MOTOR_POLE_PAIRS_INIT*aligned[0];
    return true;
}

static float clip(float x,float limit) { return fmaxf(-limit,fminf(x,limit)); }

static float pi_step(float *integral,float e,float kp,float ki,float dt,float limit,bool *limited)
{
    float cmd=*integral+kp*e;
    if (fabsf(cmd)>limit) *limited=true;
    if (fabsf(cmd)<=limit || cmd*e<0) *integral=clip(*integral+ki*dt*e,limit);
    return clip(cmd,limit);
}

static void run_test(request_t r)
{
    sensors_t s;
    if (!preflight(false,&s)) return;
    int64_t duration=llround(1e6*(double)r.cycles/r.hz);
    int64_t t0=s.time, next=0;
    if (!arm(t0,duration)) return;
    float xi_d=0,xi_q=0,xi_speed=0,id_f=0,iq_f=0,w_f=0;
    int64_t flat_since=t0;
    float previous_mod=0;
    bool limited=false;
    while (live()) {
        sensors_t previous=s;
        vTaskDelay(1);
        if (!read_sensors(&s)) break;
        /* s.u/v come from integer ADC mV. Under sustained appreciable drive,
         * two exactly unchanged readings are ambiguous: end the trial. */
        if (previous_mod<STUCK_CHECK_MOD || s.u!=previous.u || s.v!=previous.v)
            flat_since=s.time;
        else if (s.time-flat_since>=STUCK_CHECK_US) { fail("current_stuck"); break; }
        float dt,raw;
        if (!speed_sample(&s,&previous,&dt,&raw)) break;
        int elapsed=(int)(s.time-t0);
        if (elapsed>=duration) break;
        w_f+=SPEED_LPF_ALPHA*(raw-w_f);
        if (fabsf(w_f)>MAX_FILTERED_SPEED) { fail("overspeed"); break; }
        float theta=cal.direction*MOTOR_POLE_PAIRS_INIT*s.angle+cal.angle_zero+w_f*dt;
        float c=cosf(theta), sn=sinf(theta), beta=(s.u+2*s.v)*ONE_OVER_SQRT3;
        float id=s.u*c+beta*sn, iq=-s.u*sn+beta*c;
        id_f+=IAMP_FILT_ALPHA*(id-id_f); iq_f+=IAMP_FILT_ALPHA*(iq-iq_f);
        float ref=r.amp*sinf(TWO_PI_F*r.hz*elapsed*1e-6f);
        float target = ref; /* current mode injects signed rotor-frame mA directly */
        if(r.mode==SPEED) target=TORQUE_SIGN*cal.direction*
            pi_step(&xi_speed,ref-w_f,SPD_KP_MA,SPD_KI_MA,dt,IQ_LIMIT_MA,&limited);
        float d=pi_step(&xi_d,-id_f,CUR_KP_AMP_PER_MA,CUR_KI_AMP_PER_MA_S,dt,MOD_LIMIT,&limited);
        float q=pi_step(&xi_q,target-iq_f,CUR_KP_AMP_PER_MA,CUR_KI_AMP_PER_MA_S,dt,MOD_LIMIT,&limited);
        float mag=hypotf(d,q);
        if (mag>MOD_LIMIT) { d*=MOD_LIMIT/mag; q*=MOD_LIMIT/mag; limited=true; }
        previous_mod=fminf(mag,MOD_LIMIT);
        if (!apply(theta,d,q,s.time)) break;
        if (elapsed>=next) {
            if (count>=MAX_RECORDS) { fail("buffer_full"); break; }
            record[count++]=(sample_t){elapsed,s.code,ref,iq,iq_f,raw,w_f,limited};
            limited=false; /* OR of saturation events since the previous stored row */
            next=((int64_t)elapsed/RECORD_US+1)*RECORD_US;
        }
    }
    if (limited && count) record[count-1].limited=true;
}

static void motor_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        portENTER_CRITICAL(&lock);
        request_t r=request;
        portEXIT_CRITICAL(&lock);
        printf("BENCH1 BEGIN %s\n",modes[r.mode]);
        fflush(stdout);
        if(r.mode==ALIGN) align_axis(); else run_test(r);
        portENTER_CRITICAL(&lock);
        bench_io_disable();
        enabled=false;
        portEXIT_CRITICAL(&lock);
        if (!bench_io_voltage(0,0,0)) fail("pwm");
        portENTER_CRITICAL(&lock);
        const char *reason=abort_reason ? abort_reason:"ok";
        bool ok=strcmp(reason,"ok")==0;
        state=ok ? (r.mode==ALIGN ? IDLE:DONE):FAULT;
        size_t n=count;
        portEXIT_CRITICAL(&lock);
        printf("BENCH1 END %s %s %u\n",modes[r.mode],reason,(unsigned)n);
        fflush(stdout);
    }
}

static bool number(const char *s,float *value)
{
    char *end;
    errno=0;
    *value=strtof(s,&end);
    return !errno && end!=s && *end=='\0' && isfinite(*value);
}

static int error(const char *msg) { printf("BENCH1 ERR %s\n",msg); return 1; }

static int stop_cmd(int argc,char **argv)
{
    (void)argc;(void)argv;
    portENTER_CRITICAL(&lock);
    trip("stopped");
    portEXIT_CRITICAL(&lock);
    return 0;
}

static int command(int argc,char **argv)
{
    if(argc<2) return error("syntax");
    const char *op=argv[1];
    if(argc==2 && !strcmp(op,"status")) {
        portENTER_CRITICAL(&lock);
        state_t current=state;
        portEXIT_CRITICAL(&lock);
        printf("BENCH1 STATUS %s\n",states[current]); return 0;
    }
    if(argc==2 && !strcmp(op,"stop")) return stop_cmd(argc,argv);
    if(argc==2 && !strcmp(op,"ping")) {
        portENTER_CRITICAL(&lock);
        int64_t now=esp_timer_get_time();
        check_time(now); /* never revive an expired lease */
        if(active() && !abort_reason) lease=now+LEASE_US;
        portEXIT_CRITICAL(&lock); return 0;
    }
    if(argc==2 && !strcmp(op,"dump")) {
        portENTER_CRITICAL(&lock);
        state_t before=state;
        bool ok=before==DONE || before==FAULT;
        if(ok) state=DUMP;
        portEXIT_CRITICAL(&lock);
        if(!ok) return error("not_finished");
        for(size_t i=0;i<count;i++) {
            const sample_t *s=&record[i];
            printf("BENCH1 DATA %u,%d,%.6g,%.6g,%.6g,%.6g,%.6g,%d,%d\n",
                   (unsigned)i,s->t_us,s->ref,s->iq,s->iqf,s->raw,s->speed,s->code,s->limited);
        }
        printf("BENCH1 EOF %u\n",(unsigned)count); fflush(stdout);
        portENTER_CRITICAL(&lock);
        state=before;
        portEXIT_CRITICAL(&lock); return 0;
    }
    request_t r={.mode=ALIGN};
    if(!strcmp(op,"start") && argc==6) {
        float cycles;
        if(!strcmp(argv[2],"current")) r.mode=CURRENT;
        else if(!strcmp(argv[2],"speed")) r.mode=SPEED;
        else return error("mode");
        if(!number(argv[3],&r.hz) || !number(argv[4],&r.amp) || !number(argv[5],&cycles) ||
           r.hz<MIN_FREQ_HZ || r.hz>MAX_FREQ_HZ || r.amp<=0 ||
           r.amp>(r.mode==CURRENT ? IQ_LIMIT_MA:MAX_REF_SPEED) ||
           cycles<3 || cycles>12 || cycles!=floorf(cycles) || cycles/r.hz>MAX_SECONDS)
            return error("range");
        r.cycles=(int)cycles;
    } else if(strcmp(op,"cal") || argc!=2) return error("syntax");

    portENTER_CRITICAL(&lock);
    bool allowed=state==IDLE || state==DONE || (r.mode==ALIGN && (state==UNCAL || state==FAULT));
    if(allowed) {
        request=r; abort_reason=NULL; count=0;
        state=r.mode==ALIGN ? CAL:TEST;
        int64_t now=esp_timer_get_time();
        lease=now+LEASE_US;
        deadline=now+(r.mode==ALIGN ? 6000000:1000000); /* test preflight bound */
    }
    portEXIT_CRITICAL(&lock);
    if(!allowed) return error("state");
    xTaskNotifyGive(motor); return 0;
}

void app_main(void)
{
    bench_io_init();
    esp_timer_handle_t timer;
    esp_timer_create_args_t timer_args={.callback=watchdog,.name="bench_guard"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args,&timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer,2000));
    if(xTaskCreate(motor_task,"sine_motor",6144,NULL,12,&motor)!=pdPASS) abort();
    esp_console_repl_t *repl=NULL;
    esp_console_repl_config_t cfg=ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    cfg.prompt="bench>"; cfg.max_cmdline_length=96;
    esp_console_dev_uart_config_t uart=ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart,&cfg,&repl));
    const esp_console_cmd_t commands[]={
        {.command="bench",.help="status | cal | start current|speed Hz amplitude cycles | ping | stop | dump",.func=command},
        {.command="stop",.help="disable bridge now",.func=stop_cmd}};
    for(size_t i=0;i<sizeof(commands)/sizeof(commands[0]);i++)
        ESP_ERROR_CHECK(esp_console_cmd_register(&commands[i]));
    portENTER_CRITICAL(&lock);
    state=UNCAL;
    portEXIT_CRITICAL(&lock);
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
