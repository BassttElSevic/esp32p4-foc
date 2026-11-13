/* Deterministically schedule interrupts at control tick and peripheral boundaries. */
#include <setjmp.h>
#include "../../firmware/sine-bench/main/bench.c"

enum stimulus { NORMAL, NO_LINK, STALL, STOP_PREFLIGHT, STOP_PWM, ADC_FAIL,
                ADC_RAIL, SENSOR_NAN, BAD_ZERO, NOISY_ZERO, MOVING_BEFORE,
                CURRENT_TRIP, SPEED_TRIP, FLAT_SHUNTS, FROZEN_ANGLE, PWM_FAIL,
                ALIGN_LEASE_AT_BOUNDARY, FROZEN_AFTER_CAL };
static enum stimulus fault;
static int64_t clock_us, enable_time;
static bool gate;
static int enable_count, active_writes, notifications, samples;
static float applied_theta, applied_d;
static float max_d, max_q;
static bool saw_aligned[2];
static void (*timer_callback)(void *), (*task_entry)(void *);
static int64_t timer_period;
static jmp_buf idle_task;
static int (*console_bench)(int,char **), (*console_stop)(int,char **);

static int cmd(const char *op)
{
    char *args[] = {"bench",(char *)op};
    return console_bench(2,args);
}

static int start(const char *mode, const char *hz, const char *amp, const char *cycles)
{
    char *args[] = {"bench","start",(char *)mode,(char *)hz,(char *)amp,(char *)cycles};
    return console_bench(6,args);
}

int64_t esp_timer_get_time(void) { return clock_us; }
int esp_timer_create(const esp_timer_create_args_t *args,esp_timer_handle_t *timer)
{
    timer_callback=args->callback;
    *timer=(void *)1;
    return 0;
}
int esp_timer_start_periodic(esp_timer_handle_t timer,int64_t period)
{
    assert(timer);
    timer_period=period;
    return 0;
}
void xTaskNotifyGive(TaskHandle_t task)
{
    assert(task);
    ++notifications;
}
void ulTaskNotifyTake(int clear,uint32_t timeout)
{
    assert(clear && timeout==portMAX_DELAY);
    if (!notifications) longjmp(idle_task,1);
    assert(notifications==1);
    notifications=0;
}
int xTaskCreate(void (*entry)(void *),const char *name,int n,void *arg,int prio,TaskHandle_t *task)
{
    (void)name; (void)n; (void)arg; (void)prio;
    task_entry=entry;
    *task=(void *)1;
    return pdPASS;
}
int esp_console_new_repl_uart(const esp_console_dev_uart_config_t *uart,
                              const esp_console_repl_config_t *cfg,esp_console_repl_t **repl)
{
    (void)uart; (void)cfg; *repl=(void *)1; return 0;
}
int esp_console_cmd_register(const esp_console_cmd_t *c)
{
    if (!strcmp(c->command,"bench")) console_bench=c->func;
    if (!strcmp(c->command,"stop")) console_stop=c->func;
    return 0;
}
int esp_console_start_repl(esp_console_repl_t *repl) { assert(repl); return 0; }

static void advance_tick(void)
{
    clock_us+=1000;
    if (clock_us%timer_period==0) timer_callback(NULL);
    if (fault==ALIGN_LEASE_AT_BOUNDARY && gate) {
        /* Last ping exactly one lease before the transition at 1350 ms. */
        if (clock_us-enable_time<=1350000-LEASE_US) assert(cmd("ping")==0);
    } else if (clock_us%100000==0 && fault!=NO_LINK) assert(cmd("ping")==0);
}

void vTaskDelay(unsigned int ticks)
{
    assert(!lock);
    for (unsigned i=0;i<ticks;i++) advance_tick();
    if (fault==STOP_PREFLIGHT && !gate && state==TEST && !abort_reason)
        console_stop(0,NULL);
    if (fault==STALL && gate && active_writes>3) {
        int64_t before=clock_us;
        while (gate && clock_us-before <= LOOP_GAP_US+timer_period)
            advance_tick(); /* timer runs while the motor task is blocked */
        assert(!gate && clock_us-before<=LOOP_GAP_US+timer_period);
    }
}

void bench_io_init(void) { gate=false; }
void bench_io_enable(void)
{
    assert(!gate && applied_d==0 && !abort_reason);
    gate=true; enable_time=clock_us; ++enable_count; active_writes=0;
    max_d=max_q=0;
    saw_aligned[0]=saw_aligned[1]=false;
}
void bench_io_disable(void) { gate=false; }
bool bench_io_voltage(float theta,float d,float q)
{
    assert(!lock && isfinite(theta) && isfinite(d) && isfinite(q));
    assert(hypotf(d,q)<=MOD_LIMIT+1e-6f);
    if (gate) {
        ++active_writes;
        max_d=fmaxf(max_d,fabsf(d)); max_q=fmaxf(max_q,fabsf(q));
        if (state==CAL && fabsf(d-ALIGN_MOD)<1e-5f) {
            if (fabsf(theta)<1e-5f) saw_aligned[0]=true;
            if (fabsf(theta-TWO_PI_F/4)<1e-5f) saw_aligned[1]=true;
        }
    }
    if (gate && active_writes==4 && fault==STOP_PWM) {
        console_stop(0,NULL); /* between permission check and peripheral write */
        assert(!gate);
    }
    if (gate && active_writes==4 && fault==PWM_FAIL) {
        bench_io_disable();
        return false;
    }
    applied_theta=theta;
    applied_d=d;
    return true;
}
bool bench_io_sample(float *u,float *v,int *code)
{
    ++samples;
    bool late=gate && clock_us-enable_time>10000;
    bool align=gate && state==CAL;
    float theta=align ? applied_theta:0;
    float amp=align && fault!=FLAT_SHUNTS ? 500*applied_d:0;
    float a=amp*cosf(theta), b=amp*sinf(theta);
    *u=1650+a*(AMP_GAIN*R_SHUNT_OHM/SENSE_SIGN);
    *v=1650+(-a+SQRT3_F*b)*0.5f*(AMP_GAIN*R_SHUNT_OHM/SENSE_SIGN);
    *code=1000+(fault==FROZEN_ANGLE ? 0:(int)roundf(theta/MOTOR_POLE_PAIRS_INIT*AS5600_RES/TWO_PI_F));
    if (fault==BAD_ZERO) *v=700;
    if (fault==NOISY_ZERO) *u+= (samples%2)*30;
    if (fault==MOVING_BEFORE) *code+=samples%10;
    if (late) {
        if (fault==ADC_FAIL) { bench_io_disable(); return false; }
        if (fault==ADC_RAIL) *u=0;
        if (fault==SENSOR_NAN) *v=NAN;
        if (fault==CURRENT_TRIP) {
            *u=*v=1650-100; /* separately below 220 mA, inferred W exceeds it */
        }
        if (fault==SPEED_TRIP) *code+=50;
    }
    if (gate && state==TEST && fault!=FROZEN_AFTER_CAL) {
        /* Vary in-range ADC codes as a sensor stimulus, not a motor model. */
        if (fault==NORMAL) *v+=4*sinf((float)(clock_us-enable_time)*1e-5f);
    }
    return true;
}

static void run_once(void)
{
    assert(notifications==1);
    if (setjmp(idle_task)==0) task_entry(NULL);
    assert(!notifications && !gate && !enabled && !lock);
}

static void align_ok(void)
{
    fault=NORMAL;
    assert(cmd("cal")==0);
    run_once();
    assert(state==IDLE && !strcmp(modes[request.mode],"cal"));
    assert(cal.direction==1 && fabsf(cal.off_u-1650)<0.01f && !count);
    assert(saw_aligned[0] && saw_aligned[1] && max_d>0 && max_q==0);
}
static void reject_after_fault(const char *code)
{
    assert(state==FAULT && abort_reason && !strcmp(abort_reason,code));
    int before=enable_count;
    for (int i=0;i<600;i++) advance_tick();
    assert(cmd("ping")==0);
    assert(start("current","2","15","3")!=0);
    assert(!gate && !notifications && before==enable_count);
}

int main(void)
{
    app_main();
    assert(state==UNCAL && !gate && enable_count==0 && timer_callback && task_entry && console_stop);
    assert(start("speed","2","2","3")!=0);
    assert(start("current","nan","15","3")!=0);
    assert(start("current","2","15","3foo")!=0);
    align_ok();
    assert(start("current","0.5","15","4")==0);
    assert(start("speed","2","2","3")!=0 && cmd("cal")!=0 && cmd("dump")!=0);
    run_once();
    assert(state==DONE && count==2000 && max_q>0);
    for (size_t n=0;n<count;n++) {
        assert(record[n].t_us>=0 && record[n].t_us<8000000);
        assert(n==0 || record[n].t_us>record[n-1].t_us);
        assert(fabsf(record[n].ref-15*sinf(TWO_PI_F*0.5f*record[n].t_us*1e-6f))<1e-4f);
        assert(isfinite(record[n].iqf) && fabsf(record[n].ref)<=15);
    }
    assert(cmd("dump")==0 && state==DONE && !gate);
    assert(start("speed","2","2","3")==0);
    run_once();
    assert(state==DONE && count>0 && max_q>0);
    for (size_t n=0;n<count;n++)
        assert(fabsf(record[n].ref-2*sinf(TWO_PI_F*2*record[n].t_us*1e-6f))<1e-4f);

    struct {enum stimulus f; const char *reason; bool powers;} cases[]={
        {STOP_PREFLIGHT,"stopped",false},{BAD_ZERO,"bipolar_zero",false},
        {NOISY_ZERO,"noisy_zero",false},{MOVING_BEFORE,"shaft_moving",false},
        {ADC_RAIL,"adc_rail",true},{SENSOR_NAN,"adc_rail",true},
        {ADC_FAIL,"adc_or_encoder",true},{CURRENT_TRIP,"phase_overcurrent",true},
        {SPEED_TRIP,"overspeed",true},{STOP_PWM,"stopped",true},
        {PWM_FAIL,"pwm",true},{STALL,"control_timeout",true},
        {NO_LINK,"link_timeout",true}
    };
    for (size_t i=0;i<sizeof(cases)/sizeof(cases[0]);i++) {
        int before=enable_count;
        fault=cases[i].f;
        assert(start("current","2","15","3")==0);
        run_once();
        assert(enable_count==before+(int)cases[i].powers);
        reject_after_fault(cases[i].reason);
        align_ok();
    }
    fault=FLAT_SHUNTS;
    assert(cmd("cal")==0);
    run_once();
    reject_after_fault("current_polarity");
    fault=FROZEN_ANGLE;
    assert(cmd("cal")==0);
    run_once();
    reject_after_fault("angle_travel");
    /* Calibration may be aborted precisely as field orientation would change. */
    fault=ALIGN_LEASE_AT_BOUNDARY;
    int before=enable_count;
    assert(cmd("cal")==0);
    run_once();
    assert(enable_count==before+1 && saw_aligned[0] && !saw_aligned[1]);
    reject_after_fault("link_timeout");
    /* A ping exactly at an expired lease cannot authorize a stale request. */
    fault=NORMAL;
    assert(cmd("cal")==0);
    before=enable_count;
    clock_us+=LEASE_US;
    assert(cmd("ping")==0);
    run_once();
    assert(before==enable_count);
    reject_after_fault("link_timeout");
    /* A weak drive cannot reliably distinguish an unchanged ADC code from a
     * frozen INA. Preserve the low-amplitude case as an explicit limitation. */
    align_ok();
    fault=FROZEN_AFTER_CAL;
    assert(start("current","2","15","3")==0);
    run_once();
    assert(state==DONE || state==FAULT);
    if (state==DONE) {
        assert(max_q>0 && count>0);
        for (size_t n=0;n<count;n++) assert(record[n].iq==0 && record[n].iqf==0);
        puts("BENCH_TEST: valid frozen shunts during capture not detected on device.");
    }
    align_ok();
    fault=FROZEN_AFTER_CAL;
    assert(start("speed","0.5","5","3")==0);
    run_once();
    reject_after_fault("current_stuck");
    puts("BENCH_TEST_PASSED");
    return 0;
}
