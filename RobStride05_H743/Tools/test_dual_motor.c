/* The REAL production service runs against two deterministic CAN peers.
 * No firmware test commands, duplicate state machine, hardware or motion. */
#include <assert.h>
#include <stdio.h>
#include "app_motor.c"

volatile AppDebug g_debug;
MockSCB mock_scb;
typedef struct {
 uint8_t id,uid,state;
 uint32_t mode,current,accel,timeout,speed;
 uint32_t speed_writes,stops,enables;
} Peer;
static Peer peer[2];
static uint32_t tick,rx_head,rx_tail,id_writes,collisions,cancels;
static CanFrame rx_frames[128];
static bool tx_active,bus_off;
static CanStats cs;
uint32_t HAL_GetTick(void){return tick;}
uint32_t BspTime_Us(void){return tick*1000U;}
bool BspTime_DebugFrozen(void){return false;}
bool BspCan_Start(void){return true;}
bool BspCan_IsBusOff(void){return bus_off;}
bool BspCan_Recover(void){bus_off=false;return true;}
void BspCan_GetStats(CanStats *out){*out=cs;}
void BspCan_Cancel(void){if(tx_active)cancels++;tx_active=false;}
void BspCan_DiscardRx(void){rx_tail=rx_head;}
bool BspCan_Receive(CanFrame *f)
{if(rx_tail==rx_head)return false;*f=rx_frames[rx_tail++%128U];return true;}
static void enqueue(CanFrame f)
{assert(rx_head-rx_tail<128U);f.tick_ms=tick;rx_frames[rx_head++%128U]=f;cs.rx++;}
static void feedback(Peer *p)
{
 CanFrame f={0};float v=Rs05_RawFloat(p->speed);
 uint16_t encoded=(uint16_t)((v+50.0f)*65535.0f/100.0f);
 f.id=(2UL<<24)|((uint32_t)p->state<<22)|((uint32_t)p->id<<8)|APP_HOST_ID;
 f.data[0]=0x80;f.data[2]=(uint8_t)(encoded>>8);f.data[3]=(uint8_t)encoded;
 f.data[4]=0x80;f.data[6]=1;f.data[7]=44;enqueue(f);
}
static uint32_t *parameter(Peer *p,uint16_t index)
{
 switch(index) {
 case RS_RUN_MODE:return &p->mode;
 case RS_CURRENT_LIMIT:return &p->current;
 case RS_ACCEL:return &p->accel;
 case RS_TIMEOUT:return &p->timeout;
 case RS_SPEED_REF:return &p->speed;
 default:return NULL;
 }
}
bool BspCan_SendBegin(const CanFrame *f)
{
 uint32_t i,type=(f->id>>24)&31U;
 if(tx_active){collisions++;return false;}
 if(bus_off)return false;
 tx_active=true;
 for(i=0;i<2U;i++)if(peer[i].id && peer[i].id==(f->id&255U)) {
  Peer *p=&peer[i];CanFrame r={0};
  uint16_t index=(uint16_t)(f->data[0]|((uint16_t)f->data[1]<<8));
  uint32_t *value=parameter(p,index);
  if(type==0) {
   r.id=((uint32_t)p->id<<8)|0xFEU;memset(r.data,p->uid,8);enqueue(r);
  }else if(type==7) {
   p->id=(uint8_t)(f->id>>16);id_writes++;
  }else if(type==3) {
   p->state=2;p->enables++;feedback(p);
  }else if(type==4) {
   p->state=0;p->speed=0;p->stops++;feedback(p);
  }else if(type==18 && value) {
   memcpy(value,f->data+4,4);
   if(index==RS_SPEED_REF)p->speed_writes++;
  }else if(type==17) {
   uint32_t raw=value?*value:Rs05_FloatRaw(index==RS_VBUS?24.0f:0.0f);
   r.id=(17UL<<24)|((uint32_t)p->id<<8)|APP_HOST_ID;
   r.data[0]=(uint8_t)index;r.data[1]=(uint8_t)(index>>8);
   memcpy(r.data+4,&raw,4);enqueue(r);
  }
 }
 return true;
}
uint32_t BspCan_SendPoll(void)
{if(!tx_active)return CAN_TX_IDLE;tx_active=false;cs.tx_ok++;return CAN_TX_OK;}
static void run_ms(uint32_t duration)
{
 uint32_t end=tick+duration,i;
 for(;tick<end;tick+=2U) {
  if(tick%20U==0U)for(i=0;i<2U;i++)if(peer[i].id)feedback(&peer[i]);
  AppMotor_Service();
 }
}
static void setup_command(uint32_t command,bool isolated)
{
 uint32_t seq=g_debug.setup.seq+2U;
 g_debug.setup.seq=seq-1U;g_debug.setup.key=APP_ENABLE_KEY;
 g_debug.setup.command=command;g_debug.setup.old_id=127;g_debug.setup.new_id=126;
 g_debug.setup.isolated_key=isolated?APP_ISOLATED_KEY:0;g_debug.setup.seq=seq;
}
static void command(uint32_t slot,uint32_t cmd,float target)
{
 volatile MotorCommand *m=slot?&g_debug.cmd2:&g_debug.cmd;
 uint32_t seq=m->seq+2U;
 m->seq=seq-1U;m->command=cmd;m->confirm_key=APP_ENABLE_KEY;
 m->target_rad_s=target;m->speed_limit_rad_s=2;
 m->current_limit_a=0.5f;m->accel_rad_s2=5;m->seq=seq;
}
int main(void)
{
 peer[0].id=127;peer[0].uid=11;peer[1].uid=22;
 AppMotor_Init();run_ms(1100);
 assert(g_debug.motor.online && !g_debug.motor2.online);
 command(0,CMD_PREPARE,0);run_ms(40);
 assert(g_debug.motor.result==RESULT_CONFIRM && !peer[0].enables);
 setup_command(1,false);run_ms(40);
 assert(!g_debug.setup.dual_ready && g_debug.setup.result==RESULT_OFFLINE);
 setup_command(2,false);run_ms(40);assert(!id_writes);
 setup_command(2,true);run_ms(500);
 assert(id_writes==1U && peer[0].id==126 && g_debug.setup.result==RESULT_OK);
 assert(!g_debug.setup.dual_ready);
 peer[1].id=127;run_ms(1100);
 setup_command(1,false);run_ms(60);
 assert(g_debug.setup.dual_ready && g_debug.setup.result==RESULT_OK);
 command(0,CMD_PREPARE,0);command(1,CMD_PREPARE,0);run_ms(500);
 assert(g_debug.motor.state==MOTOR_PREPARED && g_debug.motor2.state==MOTOR_PREPARED);
 assert(g_debug.motor.result==0 && g_debug.motor2.result==0);
 command(0,CMD_ENABLE,0);command(1,CMD_ENABLE,0);run_ms(400);
 assert(g_debug.motor.state==MOTOR_ENABLED && g_debug.motor2.state==MOTOR_ENABLED);
 command(0,CMD_RUN,0.5f);command(1,CMD_RUN,-0.3f);run_ms(400);
 assert(g_debug.motor.state==MOTOR_RUNNING && g_debug.motor2.state==MOTOR_RUNNING);
 assert(g_debug.motor.target_rad_s==0.5f && g_debug.motor2.target_rad_s==-0.3f);
 assert(g_debug.motor.velocity_rad_s>0.45f && g_debug.motor2.velocity_rad_s< -0.25f);
 assert(g_debug.motor.current_limit_a==0.5f && g_debug.motor2.current_limit_a==0.5f);
 command(0,CMD_DISABLE,0);run_ms(100);
 assert(g_debug.motor.state==MOTOR_DISABLED && g_debug.motor2.state==MOTOR_RUNNING);
 assert(g_debug.motor2.target_rad_s==-0.3f);
 /* A disabled motor's polling cannot cancel its running peer's transmission. */
 assert(!collisions);
 g_debug.stop_all_seq=2;run_ms(100);
 assert(!g_debug.motor.enable_requested && !g_debug.motor2.enable_requested);
 assert(g_debug.motor.target_rad_s==0 && g_debug.motor2.target_rad_s==0);
 assert(peer[0].state==0 && peer[1].state==0);
 assert(!g_debug.motor.job_command && !g_debug.motor2.job_command);
 /* Shared bus failure revokes authorization and never restores motion. */
 bus_off=true;run_ms(1100);
 assert(!g_debug.setup.dual_ready && !peer[0].state && !peer[1].state);
 printf("PASS dual: isolation, ID verification, two concurrent prepare/enable/run, independent disable, all-stop, bus-off; collisions=%lu cancels=%lu\n",(unsigned long)collisions,(unsigned long)cancels);
 return 0;
}
