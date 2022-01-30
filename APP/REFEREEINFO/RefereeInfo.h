#ifndef __REFEREEINFO_H__
#define __REFEREEINFO_H__
/******************
������Ϣ�����͸������֡��װ����
update: 2017.5.7
    ��������������������
    ȫ�ֱ���˵����uart2referee.h
    ֧���ϴ�3��float����
******************/
#include "sys.h"
#include "stdio.h"  // define NULL
#include "stdbool.h" 

void float2bytes(float chosen_value, u8 * res_message);
float _bytes2float(uint8_t * chosen_Message);
void float2bytes(float chosen_value, u8 * res_message);
// flaot���ֽڻ�ת
typedef union {
    float f;
    unsigned char b[4];
} Bytes2Float;

// float��u32��ת
typedef union {
    u32 u32_value;
    unsigned char b[4];
} Bytes2U32;



// λ����Ϣ(�������ṹ�����), ��λ���ݵ�λΪ�ף�С�������λΪ��Ч���ݡ�
typedef __packed struct {
    uint8_t flag;
    float x;
    float y;
    float z;
    float compass;
}tLocData;

/*
 ����״̬���ݣ�0x0001��, ����Ƶ��1Hz��

*/

typedef __packed struct
{
uint8_t game_type : 4;
uint8_t game_progress : 4;
uint16_t stage_remain_time;
} ext_game_state_t;

/*
����������ݣ�0x0002 
����Ƶ�ʣ�������������
��С��1�ֽ�
˵����0��ƽ�֣�1���췽ʤ����2������ʤ��
*/
typedef __packed struct
{
	uint8_t winner;
} ext_game_result_t;


// �����˴������ݣ�0x0003��
typedef __packed struct
{
	//uint16_t robot_legion;
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_base_HP;
} ext_game_robot_survivors_t;


// //����ʱ�¼����ݣ�0x0101��
typedef __packed struct
{
uint32_t event_type;
} ext_event_data_t;

//����վ������ʶ��0x0102��
typedef __packed struct
{
uint8_t supply_projectile_id;
uint8_t supply_robot_id;
uint8_t supply_projectile_step;
} ext_supply_projectile_action_t;



//����վԤԼ�ӵ���0x0103��
typedef __packed struct
{
uint8_t supply_projectile_id;
uint8_t supply_num;
} ext_supply_projectile_booking_t;

//����������״̬(0x0201)
typedef __packed struct
{
uint8_t robot_id;
uint8_t robot_level;
uint16_t remain_HP;
uint16_t max_HP;
uint16_t shooter_heat0_cooling_rate;
uint16_t shooter_heat0_cooling_limit;
uint16_t shooter_heat1_cooling_rate;
uint16_t shooter_heat1_cooling_limit;
uint8_t mains_power_gimbal_output : 1;
uint8_t mains_power_chassis_output : 1;
uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

//ʵʱ�����������ݣ�0x0202��
typedef __packed struct
{
uint16_t chassis_volt;
uint16_t chassis_current;
float chassis_power;
uint16_t chassis_power_buffer;
uint16_t shooter_heat0;
uint16_t shooter_heat1;
} ext_power_heat_data_t;

//������λ�ã�0x0203��
typedef __packed struct
{
float x;
float y;
float z;
float yaw;
} ext_game_robot_pos_t;

//���������棨0x0204��
typedef __packed struct
{
uint8_t power_rune_buff;
}ext_buff_musk_t;

//���л���������״̬��0x0205��
typedef __packed struct
{
uint8_t energy_point;
uint8_t attack_time;
} aerial_robot_energy_t;

//�˺�״̬��0x0206��
typedef __packed struct
{
uint8_t armor_id : 4;
uint8_t hurt_type : 4;
} ext_robot_hurt_t;


//ʵʱ�����Ϣ��0x0207��
typedef __packed struct
{
uint8_t bullet_type;
uint8_t bullet_freq;
float bullet_speed;
} ext_shoot_data_t;


//�������ݽ�����Ϣ��0x0301��
typedef __packed struct
{
uint16_t data_cmd_id;
uint16_t send_ID;
uint16_t receiver_ID;
uint16_t blood_value;
}ext_student_interactive_header_data_t;

////////////////////////////////////////////
//�ͻ����Զ������ݣ�0x0301��������ID��data_cmd(0xD180)
typedef __packed struct
{
float data1;
float data2;
float data3;
uint8_t masks;
} client_custom_data_t;

//typedef __packed struct
//{
//uint8_t data[];
//} robot_interactive_data_t;


// ȫ�ֲ�����Ϣ�ֶζ���
extern ext_game_state_t ext_game_state;// ����������Ϣ��0x0001��
extern ext_game_state_t                           ext_game_state;// ����״̬���ݣ�0x0001��
extern ext_game_result_t                          ext_game_result;//�����������(0x0002)
extern ext_game_robot_survivors_t                 ext_game_robot_survivors;//�����˴������ݣ�0x0003��
extern ext_event_data_t                           ext_event_data;//����ʱ�¼����ݣ�0x0101��
extern ext_supply_projectile_action_t             ext_supply_projectile_action;//����վ������ʶ��0x0102��
extern ext_supply_projectile_booking_t            ext_supply_projectile_booking;//����վԤԼ�ӵ���0x0103��
extern ext_game_robot_state_t                     ext_game_robot_state;//����������״̬(0x0201)
extern ext_power_heat_data_t                      ext_power_heat_data;////ʵʱ�����������ݣ�0x0202��
extern ext_game_robot_pos_t                       ext_game_robot_pos;//������λ�ã�0x0203��
extern ext_buff_musk_t                            ext_buff_musk;//���������棨0x0204��
extern aerial_robot_energy_t                      ext_aerial_robot_energy;//���л���������״̬��0x0205��
extern ext_robot_hurt_t                           ext_robot_hurt;//�˺�״̬��0x0206��
extern ext_shoot_data_t                           ext_shoot_data;//ʵʱ�����Ϣ��0x0207��

extern ext_student_interactive_header_data_t      ext_student_interactive_header_data;//�������ݽ�����Ϣ��0x0301��

// ʹ��ǰ�ĳ�ʼ�����в�����Ϣ��ؽṹ��, ��������Ӷ����ֵ
// �ɲ��ã�ϵͳ����Ĭ�ϳ�ֵ��0����   �еĺ�����ʾ��������


// ʹ����������֡���̸���ȫ��������Ϣ��ؽṹ�塣(��У��)
// u8 frame_interpret(uint8_t * frame);
bool frame_interpret(uint8_t * _frame, uint16_t size);

// ���뵥�ֽ�������ȫ��������Ϣ��ؽṹ��, 
// �����ۻ��ֽ�Ϊһ�������ݰ�ʱ �ŵ���frame_interpret���� ��������ؽṹ�塣
void referee_info_update(uint8_t single_byte);
// �Զ�������֡, ��װ������ͷָ��custom_frame������ = 5+2+12+2 = 21
// ����ǰ��ȷ��ȫ�ֱ���MyData�ṹ���Ѹ���ֵ, 
// ����ʾ��:
// for(i=0;i<21;i++) {
//     USART_SendData(USART2, custom_frame_test[i]);
//     while(USART_GetFlagStatus(USART2, USART_FLAG_TC) != SET);
// }


// debug�õ�ȫ�ֱ���
extern u8 referee_message[64];  // ��������֡���, ����44�͹���
extern u8 cmdID;;
extern u8 blood_counter;  // (debug)�������

// ������ʱ����
//// У������֡, CRC8��CRC16
//u8 Verify_frame(uint8_t * frame);

extern void update_from_dma(void);
extern u8 seq_real;
extern u8 usart6_dma_flag;
extern int shoot_counter_referee;




/************************************************************/
typedef enum {
    game_state                  = 0x0001,     /*!< frequency = 1Hz */
    game_result                 = 0x0002,     /*!< send at game ending */
    game_robot_survivors        = 0x0003,     /*!< frequency = 1Hz */
    event_data                  = 0x0101,     /*!< send at event changing */
    supply_projectile_action    = 0x0102,     /*!< send at action */
    supply_projectile_booking   = 0x0103,     /*!< send by user, max frequency = 10Hz */
    game_robot_state            = 0x0201,     /*!< frequency = 10Hz */
    power_heat_data             = 0x0202,     /*!< frequency = 50Hz */
    game_robot_pos              = 0x0203,     /*!< frequency = 10Hz */
    buff_musk                   = 0x0204,     /*!< send at changing */
    aerial_robot_energy         = 0x0205,     /*!< frequency = 10Hz, only for aerial robot */
    robot_hurt                  = 0x0206,     /*!< send at hurting */
    shoot_data                  = 0x0207,     /*!< send at shooting */
    robot_interactive_data      = 0x0301,     /*!< send by user, max frequency = 10Hz */
} ext_cmd_id_t;
typedef __packed struct {
    uint8_t     sof;                    /*!< Fixed value 0xA5 */
    uint16_t    data_length;            /*!< Length of next data pack */
    uint8_t     seq;                    /*!< Pack sequene id */
    uint8_t     crc8;                   /*!< CRC checksum for frame header pack */
} ext_frame_header_t;

typedef enum {
    robotid_red_hero = 1,
    robotid_red_engineer = 2,
    robotid_red_infantry_1 = 3,
    robotid_red_infantry_2 = 4,
    robotid_red_infantry_3 = 5,
    robotid_red_aerial = 6,
    robotid_red_sentry = 7,
    robotid_blue_hero = 11,
    robotid_blue_engineer = 12,
    robotid_blue_infantry_1 = 13,
    robotid_blue_infantry_2 = 14,
    robotid_blue_infantry_3 = 15,
    robotid_blue_aerial = 16,
    robotid_blue_sentry = 17,

    clientid_red_hero = 0x0101,
    clientid_red_engineer = 0x0102,
    clientid_red_infantry_1 = 0x0103,
    clientid_red_infantry_2 = 0x0104,
    clientid_red_infantry_3 = 0x0105,
    clientid_red_aerial = 0x0106,
    clientid_blue_hero = 0x0111,
    clientid_blue_engineer = 0x0112,
    clientid_blue_infantry_1 = 0x0113,
    clientid_blue_infantry_2 = 0x0114,
    clientid_blue_infantry_3 = 0x0115,
    clientid_blue_aerial = 0x0116,
} ext_id_t;

typedef __packed union {
    uint8_t     masks;
    __packed struct {
        uint8_t     led1 : 1;
        uint8_t     led2 : 1;
        uint8_t     led3 : 1;
        uint8_t     led4 : 1;
        uint8_t     led5 : 1;
        uint8_t     led6 : 1;
        uint8_t     : 2;
    } masks_bits;
} ext_client_custom_data_mask_t;

typedef __packed struct {
    ext_frame_header_t              header;
    uint16_t                        cmd_id;
    
    uint16_t                        data_id;            /*!< fixed value 0xD180 */
    uint16_t                        sender_id;
    uint16_t                        client_id;
    float                           data[3];
    ext_client_custom_data_mask_t   masks;
    
    uint16_t                        crc16;
} ext_client_custom_data_t;


//???????(0x0301), ??ID:data_cmd(0x0200~0x02FF)
typedef __packed struct {
		ext_frame_header_t  header;
    uint16_t            data_id;            /*!< range 0x200~0x2FF */
    uint16_t            sender_id;
    uint16_t            robot_id;
    uint8_t             data[113];          /*!< max data length = 13byte */
} ext_robot_interactive_data_t;



typedef __packed struct {
uint8_t             operate_tpye;
uint8_t             graphic_tpye;
uint8_t             graphic_name[5];
uint8_t             layer;
uint8_t             color;
uint8_t             width;
uint16_t            start_x;
uint16_t            start_y;
uint16_t            radius;
uint16_t            end_x;
uint16_t            end_y;
int16_t             start_angle;
int16_t             end_angle;
uint8_t             text_lenght;
uint8_t             text[30];
} ext_client_graphic_draw_t;

typedef __packed struct {
		ext_frame_header_t  					header;
		uint16_t											cmd_id;
	
    uint16_t            					data_id;            /*!< range 0x200~0x2FF */
    uint16_t            					sender_id;
    uint16_t            					robot_id;
    ext_client_graphic_draw_t			graphic_data;          /*!< max data length = 13byte */
	
		uint16_t                      crc16;
} ext_robot_graphic_data_t;




#define REFEREE_FRAME_HEADER_SOF                ((uint8_t)(0xA5))

#define REFEREE_STUDENT_ROBOT_MAX               ((uint16_t)(0x0200))
#define REFEREE_STUDENT_ROBOT_MIN               ((uint16_t)(0x02FF))

#define REFEREE_STUDENT_CLIENT_SOF              ((uint16_t)(0xD180))

void referee_send_client(ext_id_t target_id, float data[3], ext_client_custom_data_mask_t masks);
void send_data_to_client(void);

void referee_send_client_graphic(ext_id_t target_id,ext_client_graphic_draw_t *graphic_draw);
void referee_clear_client_graphic(void);
void send_graphic(void);
void Send_Middle_rectangle(int level, int color,int x_length,int y_length);
void Send_SOS(void);
void Clear_SOS(void);
#endif

