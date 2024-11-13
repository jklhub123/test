#include "protocol.h"
#include <string.h>
#include "usart.h"
#include "main.h"
#include "math.h"
#include "pid.h"



uint8_t receivech;
type_cast_t kp[20],ki[20],kd[20];
type_cast_t target[20];
//uint32_t target;

uint8_t i;

struct prot_frame_parser_t
{
    uint8_t *recv_ptr;
    uint16_t r_oft;
    uint16_t w_oft;
    uint16_t frame_len;
    uint16_t found_frame_head;
};

static struct prot_frame_parser_t parser;

static uint8_t recv_buf[PROT_FRAME_LEN_RECV];

/**
  * @brief ����У���
  * @param ptr����Ҫ���������
  * @param len����Ҫ����ĳ���
  * @retval У���
  */
uint8_t check_sum(uint8_t init, uint8_t *ptr, uint8_t len )
{
  uint8_t sum = init;
  
  while(len--)
  {
    sum += *ptr;
    ptr++;
  }
  
  return sum;
}

/**
 * @brief   �õ�֡���ͣ�֡���
 * @param   *frame:  ����֡
 * @param   head_oft: ֡ͷ��ƫ��λ��
 * @return  ֡����.
 */
static uint8_t get_frame_type(uint8_t *frame, uint16_t head_oft)
{
    return (frame[(head_oft + CMD_INDEX_VAL) % PROT_FRAME_LEN_RECV] & 0xFF);
}

/**
 * @brief   �õ�֡����
 * @param   *buf:  ���ݻ�����.
 * @param   head_oft: ֡ͷ��ƫ��λ��
 * @return  ֡����.
 */
static uint16_t get_frame_len(uint8_t *frame, uint16_t head_oft)
{
    return ((frame[(head_oft + LEN_INDEX_VAL + 0) % PROT_FRAME_LEN_RECV] <<  0));    // �õ�֡����
}

/**
 * @brief   ��ȡ crc-16 У��ֵ
 * @param   *frame:  ���ݻ�����.
 * @param   head_oft: ֡ͷ��ƫ��λ��
 * @param   head_oft: ֡��
 * @return  ֡����.
 */
static uint8_t get_frame_checksum(uint8_t *frame, uint16_t head_oft, uint16_t frame_len)
{
    return (frame[(head_oft + frame_len - 1) % PROT_FRAME_LEN_RECV]);
}

/**
 * @brief   ����֡ͷ
 * @param   *buf:  ���ݻ�����.
 * @param   ring_buf_len: ��������С
 * @param   start: ��ʼλ��
 * @param   len: ��Ҫ���ҵĳ���
 * @return  -1��û���ҵ�֡ͷ������ֵ��֡ͷ��λ��.
 */
static int32_t recvbuf_find_header(uint8_t *buf, uint16_t ring_buf_len, uint16_t start, uint16_t len)
{
    uint16_t i = 0;

    for (i = 0; i < (len - 2); i++)
    {

        if (((buf[(start + i + 0) % ring_buf_len] <<  8) |
             (buf[(start + i + 1) % ring_buf_len] << 16) |
             (buf[(start + i + 2) % ring_buf_len] <<  0) ) == FRAME_HEADER)
        {
            return ((start + i) % ring_buf_len);
        }
    }
    return -1;
}

/**
 * @brief   ����Ϊ���������ݳ���
 * @param   *buf:  ���ݻ�����.
 * @param   ring_buf_len: ��������С
 * @param   start: ��ʼλ��
 * @param   end: ����λ��
 * @return  Ϊ���������ݳ���
 */
static int32_t recvbuf_get_len_to_parse(uint16_t frame_len, uint16_t ring_buf_len,uint16_t start, uint16_t end)
{
    uint16_t unparsed_data_len = 0;

    if (start <= end)
        unparsed_data_len = end - start;
    else
        unparsed_data_len = ring_buf_len - start + end;

    if (frame_len > unparsed_data_len)
        return 0;
    else
        return unparsed_data_len;
}

/**
 * @brief   ��������д�뻺����
 * @param   *buf:  ���ݻ�����.
 * @param   ring_buf_len: ��������С
 * @param   w_oft: дƫ��
 * @param   *data: ��Ҫд�������
 * @param   *data_len: ��Ҫд�����ݵĳ���
 * @return  void.
 */
static void recvbuf_put_data(uint8_t *buf, uint16_t ring_buf_len, uint16_t w_oft,
        uint8_t *data, uint16_t data_len)
{
    if ((w_oft + data_len) > ring_buf_len)               // ����������β
    {
        uint16_t data_len_part = ring_buf_len - w_oft;     // ������ʣ�೤��

        /* ���ݷ�����д�뻺����*/
        memcpy(buf + w_oft, data, data_len_part);                         // д�뻺����β
        memcpy(buf, data + data_len_part, data_len - data_len_part);      // д�뻺����ͷ
    }
    else
        memcpy(buf + w_oft, data, data_len);    // ����д�뻺����
}

/**
 * @brief   ��ѯ֡���ͣ����
 * @param   *data:  ֡����
 * @param   data_len: ֡���ݵĴ�С
 * @return  ֡���ͣ����.
 */
static uint8_t protocol_frame_parse(uint8_t *data, uint16_t *data_len)
{
    uint8_t frame_type = CMD_NONE;
    uint16_t need_to_parse_len = 0;
    int16_t header_oft = -1;
//    uint8_t checksum = 0;
    
    need_to_parse_len = recvbuf_get_len_to_parse(parser.frame_len, PROT_FRAME_LEN_RECV, parser.r_oft, parser.w_oft);    // �õ�Ϊ���������ݳ���
    if (need_to_parse_len < 5)     // �϶�������ͬʱ�ҵ�֡ͷ��֡����
        return frame_type;

    /* ��δ�ҵ�֡ͷ����Ҫ���в���*/
    if (0 == parser.found_frame_head)
    {
        /* ͬ��ͷΪ���ֽڣ����ܴ���δ���������������һ���ֽڸպ�Ϊͬ��ͷ��һ���ֽڵ������
           ��˲���ͬ��ͷʱ�����һ���ֽڽ���������Ҳ���ᱻ����*/
        header_oft = recvbuf_find_header(parser.recv_ptr, PROT_FRAME_LEN_RECV, parser.r_oft, need_to_parse_len);
        if (0 <= header_oft)
        {
            /* ���ҵ�֡ͷ*/
            parser.found_frame_head = 1;
            parser.r_oft = header_oft;
          
            /* ȷ���Ƿ���Լ���֡��*/
            if (recvbuf_get_len_to_parse(parser.frame_len, PROT_FRAME_LEN_RECV,
                    parser.r_oft, parser.w_oft) < 5)
                return frame_type;
        }
        else 
        {
            /* δ��������������Ȼδ�ҵ�֡ͷ�������˴ν���������������*/
            parser.r_oft = ((parser.r_oft + need_to_parse_len - 2) % PROT_FRAME_LEN_RECV);
            return frame_type;
        }
    }
    
    /* ����֡������ȷ���Ƿ���Խ������ݽ���*/
    if (0 == parser.frame_len) 
    {
        parser.frame_len = get_frame_len(parser.recv_ptr, parser.r_oft);
        if(need_to_parse_len < parser.frame_len)
            return frame_type;
    }

//    /* ֡ͷλ��ȷ�ϣ���δ���������ݳ���֡�������Լ���У���*/
//    if ((parser.frame_len + parser.r_oft - PROT_FRAME_LEN_CHECKSUM) > PROT_FRAME_LEN_RECV)
//    {
//        /* ����֡����Ϊ�����֣�һ�����ڻ�����β��һ�����ڻ�����ͷ */
//        checksum = check_sum(checksum, parser.recv_ptr + parser.r_oft, 
//                PROT_FRAME_LEN_RECV - parser.r_oft);
//        checksum = check_sum(checksum, parser.recv_ptr, parser.frame_len -
//                PROT_FRAME_LEN_CHECKSUM + parser.r_oft - PROT_FRAME_LEN_RECV);
//    }
//    else 
//    {
//        /* ����֡����һ����ȡ��*/
//        checksum = check_sum(checksum, parser.recv_ptr + parser.r_oft, parser.frame_len - PROT_FRAME_LEN_CHECKSUM);
//    }

//    if (checksum == get_frame_checksum(parser.recv_ptr, parser.r_oft, parser.frame_len))
//    {
        /* У��ɹ���������֡���� */
        if ((parser.r_oft + parser.frame_len) > PROT_FRAME_LEN_RECV) 
        {
            /* ����֡����Ϊ�����֣�һ�����ڻ�����β��һ�����ڻ�����ͷ*/
            uint16_t data_len_part = PROT_FRAME_LEN_RECV - parser.r_oft;
            memcpy(data, parser.recv_ptr + parser.r_oft, data_len_part);
            memcpy(data + data_len_part, parser.recv_ptr, parser.frame_len - data_len_part);
        }
        else 
        {
            /* ����֡����һ����ȡ��*/
            memcpy(data, parser.recv_ptr + parser.r_oft, parser.frame_len);
        }
        *data_len = parser.frame_len;
        frame_type = get_frame_type(parser.recv_ptr, parser.r_oft);

        /* �����������е�����֡*/
        parser.r_oft = (parser.r_oft + parser.frame_len) % PROT_FRAME_LEN_RECV;
//    }
//    else
//    {
//        /* У�����˵��֮ǰ�ҵ���֡ͷֻ��żȻ���ֵķ�����*/
//        parser.r_oft = (parser.r_oft + 1) % PROT_FRAME_LEN_RECV;
//    }
    parser.frame_len = 0;
    parser.found_frame_head = 0;

    return frame_type;
}

/**
 * @brief   �������ݴ���
 * @param   *data:  Ҫ��������ݵ�����.
 * @param   data_len: ���ݵĴ�С
 * @return  void.
 */
void protocol_data_recv(uint8_t *data, uint16_t data_len)
{
    recvbuf_put_data(parser.recv_ptr, PROT_FRAME_LEN_RECV, parser.w_oft, data, data_len);    // ��������
    parser.w_oft = (parser.w_oft + data_len) % PROT_FRAME_LEN_RECV;                          // ����дƫ��
		//printf("1");
}

/**
 * @brief   ��ʼ������Э��
 * @param   void
 * @return  ��ʼ�����.
 */
int32_t protocol_init(void)
{
    memset(&parser, 0, sizeof(struct prot_frame_parser_t));
    
    /* ��ʼ���������ݽ��������������*/
    parser.recv_ptr = recv_buf;
    return 0;
}

/**
 * @brief   ���յ����ݴ���
 * @param   void
 * @return  -1��û���ҵ�һ����ȷ������.
 */
int8_t receiving_process(void)
{
  uint8_t frame_data[256];         // Ҫ�ܷ������֡
  uint16_t frame_len = 0;          // ֡����
  uint8_t cmd_type = CMD_NONE;     // ��������
  packet_head_t packet;
	uint32_t a[3] = {0};
	uint32_t b[3] = {0};
	
//  int16_t angdata=0;
	
  while(1)
  {
    cmd_type = protocol_frame_parse(frame_data, &frame_len);
    switch (cmd_type)
    {
      case CMD_NONE:
      {
        return -1;
      }

      case SET_P_I_D_CMD:
      {
				receivech = frame_data[CHX_INDEX_VAL];
        kp[receivech].i = COMPOUND_32BIT(&frame_data[9]);
				ki[receivech].i = COMPOUND_32BIT(&frame_data[13]);
				kd[receivech].i = COMPOUND_32BIT(&frame_data[17]);
				updateparameter(receivech,kp[receivech].f,ki[receivech].f,kd[receivech].f);
      }
      break;

      case SET_TARGET_CMD:
      {	
				receivech = frame_data[CHX_INDEX_VAL];	
				target[receivech].i = COMPOUND_32BIT(&frame_data[9]);
//				b[0]=target.i;
//				HAL_UART_Transmit(&huart1, (uint8_t *)b, 4, 0xFFFFF);
//				printf("%d\r\n",receivech);
//				printf("%.3f\r\n",target.f);
      }
      break;
      
      case START_CMD:
      { 
				i = 1;		
      }
      break;
			
      case STOP_CMD:
      { 
				i = 0;
      }
      break;
      
      case RESET_CMD:
      {
         HAL_NVIC_SystemReset();          // ��λϵͳ
      }
      break;

      default: 
        return -1;
    }
  }
}

/**
  * @brief ������λ����ֵ
  * @param cmd������
  * @param ch: ����ͨ��
  * @param data������ָ��
  * @param num����������
  * @retval ��
  */
void set_computer_value(uint8_t cmd, uint8_t ch, void *data, uint8_t num)
{
//    uint8_t sum = 0;    // У���
    num *= 2;           // һ������ 2 ���ֽ�
  
    static packet_head_t set_packet;
  
    set_packet.head = FRAME_HEADER>>8;     // ��ͷ
	  set_packet.head_t = FRAME_HEADER&0xFF;
    set_packet.len  = 0x06 + num;      // ����
    set_packet.ch   = ch;              // ����ͨ��
    set_packet.cmd  = cmd;             // ��������
   
    HAL_UART_Transmit(&huart1, (uint8_t *)&set_packet, sizeof(set_packet), 0xFFFFF);    // ��������ͷ
    HAL_UART_Transmit(&huart1, (uint8_t *)data, num, 0xFFFFF);                          // ���Ͳ���
    
}
/**
  * @brief ������λ����ֵ(32λ)
  * @param cmd������
  * @param ch: ����ͨ��
  * @param data������ָ��
  * @param num����������
  * @retval ��
  */
void set_computer_32value(uint8_t cmd, uint8_t ch, void *data, uint8_t num)
{
//    uint8_t sum = 0;    // У���
    num *= 4;           // һ������ 2 ���ֽ�
  
    static packet_head_t set_packet;
  
    set_packet.head = FRAME_HEADER>>8;     // ��ͷ
	  set_packet.head_t = FRAME_HEADER&0xFF;
    set_packet.len  = 0x06 + num;      // ����
    set_packet.ch   = ch;              // ����ͨ��
    set_packet.cmd  = cmd;             // ��������
   
    HAL_UART_Transmit(&huart1, (uint8_t *)&set_packet, sizeof(set_packet), 0xFFFFF);    // ��������ͷ
    HAL_UART_Transmit(&huart1, (uint8_t *)data, num, 0xFFFFF);                          // ���Ͳ���
    
}


/**********************************************************************************************/
