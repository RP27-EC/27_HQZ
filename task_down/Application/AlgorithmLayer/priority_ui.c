/* priority_ui.c - UI 优先级调度 */
#include "priority_ui.h"
#include <stdio.h>
#include "rc_sensor.h"
 
 
#define AUTO_UI_NAME_ENABLE
 
#define HIGH_PRIORITY_WEIGHT 1000
#define MID_PRIORITY_WEIGHT  500
#define LOW_PRIORITY_WEIGHT  0

#define HIGH_CHAR_PRIORITY_LEVEL 7
#define MID_CHAR_PRIORITY_LEVEL  5

#define SEND_INTERVAL        100
#define PER_INIT_UI_TIMES    1

#define UI_GRAPHIC_MAX_PER_FRAME 7

static ui_info_t *pending_graphic_ui[UI_GRAPHIC_MAX_PER_FRAME];
static uint8_t pending_graphic_num = 0;

static ui_info_t *pending_character_ui = NULL;

static void Ui_Clear_Pending(void);
static void Ui_Commit_Pending(void);
static void Ui_Rollback_Pending(void);
static uint8_t Send_Graphic_Buffer( const ext_client_custom_graphic_seven_t *buffer,  uint8_t graphic_num);

/* Init_Ui_Condition */
bool Init_Ui_Condition(void)
{
    static bool first_init_finished = false;
    static uint8_t rc_status_last = DEV_OFFLINE;
    static uint8_t s1_value_last = 0;
    static uint32_t last_rc_offline_time = 0;

    uint8_t rc_online_rising_edge = 0;
    uint8_t s1_changed = 0;

     
    if (!first_init_finished && HAL_GetTick() >= 100)
    {
        first_init_finished = true;

        rc_status_last = rc_dev.work_state;
        if (rc_dev.info != NULL)
        {
            s1_value_last = rc_dev.info->s1.value;
        }

        return true;
    }

     
    if (rc_dev.work_state == DEV_ONLINE &&
        rc_status_last == DEV_OFFLINE)
    {
        rc_online_rising_edge = 1;
        last_rc_offline_time = 0;
    }

     
    if (rc_dev.work_state == DEV_OFFLINE)
    {
        if (last_rc_offline_time == 0)
        {
            last_rc_offline_time = HAL_GetTick();
        }
        else if ((HAL_GetTick() - last_rc_offline_time) >= 10000U)
        {
            last_rc_offline_time = HAL_GetTick();

            rc_status_last = rc_dev.work_state;
            return true;
        }
    }
    else
    {
        last_rc_offline_time = 0;
    }

     
    if (rc_dev.info != NULL)
    {
        if (s1_value_last != rc_dev.info->s1.value)
        {
            s1_changed = 1;
        }

        s1_value_last = rc_dev.info->s1.value;
    }

    rc_status_last = rc_dev.work_state;

    return rc_online_rising_edge || s1_changed;
}

 

     uint32_t Calculate_Priority(ui_info_t *msg);                                     
     Node_u *SortedMerge(Node_u *a, Node_u *b);     
     void FrontBackSplit(Node_u *source, Node_u **frontRef, Node_u **backRef);
     void mergeSort(Node_u **headRef);    
     ui_status_e Store_High_Priority_UI(Node_u* dynamic_list_head,Node_u* const_list_head,ui_info_t* graphic_priority_buffer, ui_info_t* character_priority_buffer, uint8_t* ui_graphic_buffer_num,ui_send_mode_e *ui_send_mode);


     ui_status_e Init_Priority_LinkedList(Node_u** headRef, ui_info_t *ui_input, uint8_t num); 
     ui_status_e Init_Type_LinkedLists(Node_u** graphic_link, Node_u** char_link, ui_info_t *dynamic_ui_info, ui_info_t *const_ui_info, uint8_t dynamic_num, uint8_t const_num); 

     ext_client_custom_character_t Process_Char_Info_To_Buffer(ui_info_t ui_info, uint8_t add_operate_enable); 
     ext_client_custom_graphic_seven_t Process_Graphic_To_Buffer(ui_info_t *ui_info, uint8_t ui_info_size, uint8_t add_operate_enable);

     ui_status_e Ui_Send_Normal();
     ui_status_e Ui_Send_Add();

     ui_status_e Init_Ui_List(ui_info_t *dynamic_ui_info, uint8_t dynamic_ui_num, ui_info_t *const_ui_info, uint8_t const_ui_num);
     void Ui_Send();
      ui_status_e Enqueue_Ui_For_Sending(ui_info_t *ui_info);

  





 

/* Calculate_Priority */
uint32_t Calculate_Priority(ui_info_t *msg) 
{
  uint32_t priority_value = 0;
  uint32_t currentTick = HAL_GetTick();
  uint32_t age = currentTick - msg->updateTick;

  if(msg->ui_config.priority == HIGH_PRIORITY)
  {
	 
    priority_value = HIGH_PRIORITY_WEIGHT;
  }
  else if(msg->ui_config.priority == MID_PRIORITY)
  {
    priority_value = MID_PRIORITY_WEIGHT;
  }
  else
  {
    priority_value = LOW_PRIORITY_WEIGHT;
  }

  if(msg->sent_state == MESSAGE_NOT_SENT)
  {
    priority_value += age ;
  }

  return priority_value;
}
/* 接口说明 */
Node_u* SortedMerge(Node_u* a, Node_u* b)
{
  Node_u* res = NULL;

  if (a == NULL)
    return (b);
  else if (b == NULL)
    return (a);

  a->ui->priority_value = Calculate_Priority(a->ui);
  b->ui->priority_value = Calculate_Priority(b->ui);

  if (a->ui->sent_state == MESSAGE_NOT_SENT && b->ui->sent_state == MESSAGE_SENT)
  {
    res = a;
    res->next = SortedMerge(a->next, b);
  }
  else if (a->ui->sent_state == MESSAGE_SENT && b->ui->sent_state == MESSAGE_NOT_SENT)
  {
    res = b;
    res->next = SortedMerge(a, b->next);
  }
  else if (a->ui->priority_value >= b->ui->priority_value) 
  {
    res = a;
    res->next = SortedMerge(a->next, b);
  }
  else 
  {
    res = b;
    res->next = SortedMerge(a, b->next);
  }

  return res;
}

/* FrontBackSplit */
void FrontBackSplit(Node_u* source, Node_u** frontRef, Node_u** backRef)
{
   Node_u* fast;
   Node_u* slow;
   slow = source;
   fast = source->next;
 
   while (fast != NULL) {
      fast = fast->next;
      if (fast != NULL) {
        slow = slow->next;
        fast = fast->next;
      }
   }
 
   *frontRef = source;
   *backRef = slow->next;
   slow->next = NULL;
}

/* mergeSort */
void mergeSort(Node_u** headRef)
{
   Node_u* head = *headRef;
   Node_u* a;
   Node_u* b;
 
   if ((head == NULL) || (head->next == NULL)) {
      return;
   }
 
   FrontBackSplit(head, &a, &b);
 
   mergeSort(&a);
   mergeSort(&b);
 
   *headRef = SortedMerge(a, b);
}

/* 接口说明 */
ui_status_e Init_Priority_LinkedList(Node_u** headRef, ui_info_t *ui_input, uint8_t num)
{
  if (headRef == NULL)
  {
    return UI_ERROR;
  }
  
  Node_u* newNode = NULL;
  Node_u* cursor = *headRef;
  ui_info_t *ui_ptr = ui_input;
  ui_info_t zero_struct;
  memset(&zero_struct, 0, sizeof(ui_info_t));
  for (uint8_t i = 0; i < num; i++) 
  {
    if (memcmp(ui_ptr, &zero_struct, sizeof(ui_info_t)) != 0)
    {
      newNode = (Node_u*)malloc(sizeof(Node_u));
      if (newNode == NULL)
      {
        return UI_ERROR;
      }
      newNode->ui = ui_ptr;
      newNode->next = NULL;

      if (*headRef == NULL)
      {
        *headRef = newNode;
      }
      else
      {
        cursor->next = newNode;
      }
      cursor = newNode;
    }
    ui_ptr++;
  }
  return UI_OK;
}

/* 接口说明 */
ui_status_e Init_Type_LinkedLists(Node_u** graphic_link, Node_u** char_link, ui_info_t *dynamic_ui_info, ui_info_t *const_ui_info, uint8_t dynamic_num, uint8_t const_num)
{
  if (graphic_link == NULL || char_link == NULL)
  {
    return UI_ERROR;
  }

  Node_u* graphic_link_cursor = NULL;
  Node_u* char_link_cursor = NULL;
  
  ui_info_t zero_struct;
  memset(&zero_struct, 0, sizeof(ui_info_t));

  ui_info_t *dynamic_ptr = dynamic_ui_info;
  for (uint8_t i = 0; i < dynamic_num; i++) 
  {
    if (memcmp(dynamic_ptr, &zero_struct, sizeof(ui_info_t)) == 0)
    {
      dynamic_ptr++;
      continue;
    }
    Node_u* newNode = (Node_u*)malloc(sizeof(Node_u));
    if (newNode == NULL)
    {
      return UI_ERROR;
    }
    newNode->ui = dynamic_ptr;
    newNode->next = NULL;
    #ifdef AUTO_UI_NAME_ENABLE
      char *name = dynamic_ptr->ui_config.name;
      sprintf(name, "%d", i);
    #endif
    if (dynamic_ptr->ui_config.ui_type != CHAR && dynamic_ptr->ui_config.operate_type != DELETE ) 
    {
      if (*graphic_link == NULL)
      {
        *graphic_link = newNode;
      } 
      else
      {
        graphic_link_cursor->next = newNode;
      }
      graphic_link_cursor = newNode;
    } 
    else if (dynamic_ptr->ui_config.operate_type != DELETE)
    {
      if (*char_link == NULL)
      {
        *char_link = newNode;
      } 
      else
      {
        char_link_cursor->next = newNode;
      }
      char_link_cursor = newNode;
    }
    dynamic_ptr++;
  }

  ui_info_t *const_ptr = const_ui_info;
  for (uint8_t i = 0; i < const_num; i++) 
  {
    if (memcmp(const_ptr, &zero_struct, sizeof(ui_info_t)) == 0)
    {
      const_ptr++;
      continue;
    }
    Node_u* newNode = (Node_u*)malloc(sizeof(Node_u));
    if (newNode == NULL)
    {
      return UI_ERROR;
    }
    newNode->ui = const_ptr;
    newNode->next = NULL;
    #ifdef AUTO_UI_NAME_ENABLE
      char *name = const_ptr->ui_config.name;
      sprintf(name, "%d", i + dynamic_num + 1);
    #endif
    if (const_ptr->ui_config.ui_type != CHAR && const_ptr->ui_config.operate_type != DELETE) 
    {
      if (*graphic_link == NULL)
      {
        *graphic_link = newNode;
      } 
      else
      {
        graphic_link_cursor->next = newNode;
      }
      graphic_link_cursor = newNode;
    } 
    else if(const_ptr->ui_config.operate_type != DELETE)
    {
      if (*char_link == NULL)
      {
        *char_link = newNode;
      }
      else
      {
        char_link_cursor->next = newNode;
      }
      char_link_cursor = newNode;
    }
    const_ptr++;
  }
  return UI_OK;
}


/* 接口说明 */
ui_status_e Store_High_Priority_UI(
    Node_u *dynamic_list_head,
    Node_u *const_list_head,
    ui_info_t *graphic_priority_buffer,
    ui_info_t *character_priority_buffer,
    uint8_t *ui_graphic_buffer_num,
    ui_send_mode_e *ui_send_mode)
{
    (void)const_list_head;

    if (graphic_priority_buffer == NULL ||
        character_priority_buffer == NULL ||
        ui_graphic_buffer_num == NULL ||
        ui_send_mode == NULL)
    {
        return UI_ERROR;
    }

    Ui_Clear_Pending();

    memset(graphic_priority_buffer,
           0,
           sizeof(ui_info_t) * UI_GRAPHIC_MAX_PER_FRAME);

    *ui_graphic_buffer_num = 0;

    if (dynamic_list_head == NULL)
    {
        return UI_ERROR;
    }

    Node_u *cursor = dynamic_list_head;

     
    while (cursor != NULL)
    {
        if (cursor->ui != NULL &&
            cursor->ui->sent_state == MESSAGE_NOT_SENT)
        {
            if (cursor->ui->ui_config.ui_type == CHAR)
            {
                *character_priority_buffer = *cursor->ui;
                pending_character_ui = cursor->ui;
                *ui_send_mode = SEND_CHAR_MODE;

                return UI_OK;
            }

             
            break;
        }

        cursor = cursor->next;
    }

     
    cursor = dynamic_list_head;

    while (cursor != NULL &&
           *ui_graphic_buffer_num < UI_GRAPHIC_MAX_PER_FRAME)
    {
        if (cursor->ui != NULL &&
            cursor->ui->sent_state == MESSAGE_NOT_SENT &&
            cursor->ui->ui_config.ui_type != CHAR)
        {
            uint8_t index = *ui_graphic_buffer_num;

            graphic_priority_buffer[index] = *cursor->ui;
            pending_graphic_ui[index] = cursor->ui;

            (*ui_graphic_buffer_num)++;
        }

        cursor = cursor->next;
    }

    pending_graphic_num = *ui_graphic_buffer_num;

    if (*ui_graphic_buffer_num > 0)
    {
        *ui_send_mode = SEND_GRAPHIC_MODE;
        return UI_OK;
    }

     
    cursor = dynamic_list_head;

    while (cursor != NULL)
    {
        if (cursor->ui != NULL &&
            cursor->ui->sent_state == MESSAGE_NOT_SENT &&
            cursor->ui->ui_config.ui_type == CHAR)
        {
            *character_priority_buffer = *cursor->ui;
            pending_character_ui = cursor->ui;
            *ui_send_mode = SEND_CHAR_MODE;

            return UI_OK;
        }

        cursor = cursor->next;
    }

     
    return UI_BUSY;
}


/* add_operate_enable */
ext_client_custom_character_t Process_Char_Info_To_Buffer(
    ui_info_t ui_info,
    uint8_t add_operate_enable)
{
    char text_buffer[30] = {0};
    uint16_t length = 0;

    memcpy(text_buffer,
           ui_info.ui_config.text,
           sizeof(text_buffer));

    while (length < sizeof(text_buffer) &&
           text_buffer[length] != '\00')
    {
        length++;
    }

    graphic_data_struct_t char_buff =
        draw_char(
            ui_info.ui_config.name,
            add_operate_enable ? ADD
                               : ui_info.ui_config.operate_type,
            ui_info.ui_config.layer,
            ui_info.ui_config.color,
            ui_info.ui_config.size,
            length,
            ui_info.ui_config.width,
            ui_info.ui_config.start_x,
            ui_info.ui_config.start_y);

    ext_client_custom_character_t res = {0};

    res.grapic_data_struct = char_buff;

    memcpy(res.data,
           text_buffer,
           sizeof(res.data));

    return res;
}

/* 接口说明 */
ext_client_custom_graphic_seven_t Process_Graphic_To_Buffer(ui_info_t *ui_info, uint8_t ui_info_size, uint8_t add_operate_enable)
{
  ext_client_custom_graphic_seven_t res = {0};
  ui_info_t *ui_ptr = ui_info;
  if (ui_info == NULL)
  {
      return res;
  }

  if (ui_info_size > UI_GRAPHIC_MAX_PER_FRAME)
  {
      ui_info_size = UI_GRAPHIC_MAX_PER_FRAME;
  }

  for (uint8_t i = 0; i < ui_info_size; i++)
  {
    operate_tpye_e operate_tpye;
    if (add_operate_enable == 0)
    {
      operate_tpye = ui_ptr->ui_config.operate_type;
    }
    else
    {
      operate_tpye = ADD;
    }
    char *name = ui_ptr->ui_config.name;
    uint8_t layer = ui_ptr->ui_config.layer;
    uint8_t color = ui_ptr->ui_config.color;
    uint16_t width = ui_ptr->ui_config.width;
    uint16_t start_x = ui_ptr->ui_config.start_x;
    uint16_t start_y = ui_ptr->ui_config.start_y;
    uint16_t end_x = ui_ptr->ui_config.end_x;        
    uint16_t end_y = ui_ptr->ui_config.end_y;       
    uint16_t radius = ui_ptr->ui_config.radius;      
    uint16_t start_angel = ui_ptr->ui_config.start_angel; 
    uint16_t end_angel = ui_ptr->ui_config.end_angel;   
    uint16_t size = ui_ptr->ui_config.size;
    float float_num = ui_ptr->ui_config.float_num; 
    uint16_t decimal = ui_ptr->ui_config.decimal;
    int32_t int_num = ui_ptr->ui_config.int_num;      
    switch (ui_ptr->ui_config.ui_type)
    {
    case LINE:
      res.grapic_data_struct[i] = draw_line(name,
                                            operate_tpye,
                                            layer,
                                            color,
                                            width,
                                            start_x,
                                            start_y,
                                            end_x,
                                            end_y);
      break;
    case CIRCLE:
      res.grapic_data_struct[i] = draw_circle(name,
                                              operate_tpye,
                                              layer,
                                              color,
                                              width,
                                              start_x,
                                              start_y,
                                              radius);
      break;
    case RECTANGEL:
      res.grapic_data_struct[i] = draw_rectangle(name,
                                                 operate_tpye,
                                                 layer,
                                                 color,
                                                 width,
                                                 start_x,
                                                 start_y,
                                                 end_x,
                                                 end_y);
      break;
    case ELLIPSE:
      res.grapic_data_struct[i] = draw_ellipse(name,
                                               operate_tpye,
                                               layer,
                                               color,
                                               width,
                                               start_x,
                                               start_y,
                                               end_x,
                                               end_y);
      break;
    case ARC:
      res.grapic_data_struct[i] = draw_arc(name,
                                           operate_tpye,
                                           layer,
                                           color,
                                           start_angel,
                                           end_angel,
                                           width,
                                           start_x,
                                           start_y,
                                           end_x,
                                           end_y);
      break;
    case FLOAT:
      res.grapic_data_struct[i] = draw_float(name,
                                             operate_tpye,
                                             layer,
                                             color,
                                             size,
                                             decimal,
                                             width,
                                             start_x,
                                             start_y,
                                             (int32_t) (float_num *1000));
      break;
    case INT:
      res.grapic_data_struct[i] = draw_int(name,
                                           operate_tpye,
                                           layer,
                                           color,
                                           size,
                                           width,
                                           start_x,
                                           start_y,
                                           int_num);
      break;
    default:
      break;
    }
    ui_ptr ++;
  }
  return res;
}

static uint8_t Send_Graphic_Buffer(
    const ext_client_custom_graphic_seven_t *buffer,
    uint8_t graphic_num)
{
    if (buffer == NULL || graphic_num == 0)
    {
        return HAL_ERROR;
    }

    if (graphic_num == 1)
    {
        ext_client_custom_graphic_single_t tx = {0};

        tx.grapic_data_struct =
            buffer->grapic_data_struct[0];

        return client_send_single_graphic(tx);
    }

    if (graphic_num == 2)
    {
        ext_client_custom_graphic_double_t tx = {0};

        memcpy(tx.grapic_data_struct,
               buffer->grapic_data_struct,
               sizeof(tx.grapic_data_struct));

        return client_send_double_graphic(tx);
    }

    if (graphic_num <= 5)
    {
        ext_client_custom_graphic_five_t tx = {0};

        memcpy(tx.grapic_data_struct,
               buffer->grapic_data_struct,
               graphic_num * sizeof(graphic_data_struct_t));

        return client_send_five_graphic(tx);
    }

     
    return client_send_seven_graphic(*buffer);
}













 
Node_u *dynamic_list_head = NULL;
Node_u *const_list_head   = NULL;
Node_u *graphic_list_head = NULL;
Node_u *char_list_head    = NULL;

ui_info_t graphic_priority_buffer[7];
ui_info_t character_priority_buffer;

ui_send_mode_e ui_send_mode;
uint8_t ui_graphic_buffer_num = 0;

/* dynamic_ui_num */
ui_status_e Init_Ui_List(
    ui_info_t *dynamic_ui_info,
    uint8_t dynamic_ui_num,
    ui_info_t *const_ui_info,
    uint8_t const_ui_num)
{
    ui_status_e res1;
    ui_status_e res2;
    ui_status_e res3;

    res1 = Init_Priority_LinkedList(
        &dynamic_list_head,
        dynamic_ui_info,
        dynamic_ui_num);

    if (res1 != UI_OK)
    {
        return UI_ERROR;
    }

    res2 = Init_Priority_LinkedList(
        &const_list_head,
        const_ui_info,
        const_ui_num);

    if (res2 != UI_OK)
    {
        return UI_ERROR;
    }

    res3 = Init_Type_LinkedLists(
        &graphic_list_head,
        &char_list_head,
        dynamic_ui_info,
        const_ui_info,
        dynamic_ui_num,
        const_ui_num);

    if (res3 != UI_OK)
    {
        return UI_ERROR;
    }

    return UI_OK;
}

/* 接口说明 */
ui_status_e Ui_Send_Normal(void)
{
    ext_client_custom_graphic_seven_t graphic_tx_buffer = {0};
    ext_client_custom_character_t character_tx_buffer = {0};

    uint8_t send_result = HAL_ERROR;

     
    mergeSort(&dynamic_list_head);

    ui_status_e store_result =
        Store_High_Priority_UI(
            dynamic_list_head,
            const_list_head,
            graphic_priority_buffer,
            &character_priority_buffer,
            &ui_graphic_buffer_num,
            &ui_send_mode);

     
    if (store_result == UI_BUSY)
    {
        Ui_Clear_Pending();
        return UI_OK;
    }

    if (store_result != UI_OK)
    {
        Ui_Rollback_Pending();
        return UI_ERROR;
    }

    if (ui_send_mode == SEND_CHAR_MODE)
    {
        character_tx_buffer =
            Process_Char_Info_To_Buffer(
                character_priority_buffer,
                0);

        send_result =
            client_send_char(character_tx_buffer);
    }
    else if (ui_send_mode == SEND_GRAPHIC_MODE)
    {
        if (ui_graphic_buffer_num == 0)
        {
            Ui_Rollback_Pending();
            return UI_OK;
        }

        graphic_tx_buffer =
            Process_Graphic_To_Buffer(
                graphic_priority_buffer,
                ui_graphic_buffer_num,
                0);

        send_result =
            Send_Graphic_Buffer(
                &graphic_tx_buffer,
                ui_graphic_buffer_num);
    }
    else
    {
        Ui_Rollback_Pending();
        return UI_ERROR;
    }

     
    if (send_result == HAL_OK)
    {
        Ui_Commit_Pending();
        return UI_OK;
    }

     
    Ui_Rollback_Pending();

    if (send_result == HAL_BUSY)
    {
        return UI_BUSY;
    }

    return UI_ERROR;
}

/* uint8_t */
ui_status_e Ui_Send_Add(void)
{
    static uint8_t is_send_char_finish_flag = false;
    static uint8_t is_send_graphic_finish_flag = false;

    static Node_u *graphic_list_cursor = NULL;
    static Node_u *char_list_cursor = NULL;

    if (graphic_list_head == NULL)
    {
        is_send_graphic_finish_flag = true;
    }

    if (char_list_head == NULL)
    {
        is_send_char_finish_flag = true;
    }

    if (graphic_list_cursor == NULL &&
        graphic_list_head != NULL &&
        !is_send_graphic_finish_flag)
    {
        graphic_list_cursor = graphic_list_head;
    }

    if (char_list_cursor == NULL &&
        char_list_head != NULL &&
        !is_send_char_finish_flag)
    {
        char_list_cursor = char_list_head;
    }

     
    if (char_list_cursor != NULL &&
        !is_send_char_finish_flag)
    {
        ext_client_custom_character_t character_tx_buffer =
            Process_Char_Info_To_Buffer(
                *char_list_cursor->ui,
                1);

        uint8_t send_result =
            client_send_char(character_tx_buffer);

        if (send_result == HAL_BUSY)
        {
            return UI_BUSY;
        }

        if (send_result != HAL_OK)
        {
            return UI_ERROR;
        }

        if (char_list_cursor->next == NULL)
        {
            is_send_char_finish_flag = true;
            char_list_cursor = NULL;
        }
        else
        {
            char_list_cursor = char_list_cursor->next;
        }

        return UI_BUSY;
    }

     
    if (graphic_list_cursor != NULL &&
        !is_send_graphic_finish_flag)
    {
        ui_info_t graphic_info_buffer[UI_GRAPHIC_MAX_PER_FRAME] = {0};
        ext_client_custom_graphic_seven_t graphic_tx_buffer = {0};

        Node_u *read_cursor = graphic_list_cursor;
        Node_u *next_group_cursor = graphic_list_cursor;

        uint8_t count = 0;
        uint8_t reached_end = false;

        while (read_cursor != NULL &&
               count < UI_GRAPHIC_MAX_PER_FRAME)
        {
            graphic_info_buffer[count] = *read_cursor->ui;
            count++;

            if (read_cursor->next == NULL)
            {
                reached_end = true;
                next_group_cursor = NULL;
                break;
            }

            read_cursor = read_cursor->next;
            next_group_cursor = read_cursor;
        }

        graphic_tx_buffer =
            Process_Graphic_To_Buffer(
                graphic_info_buffer,
                count,
                1);

        uint8_t send_result =
            Send_Graphic_Buffer(
                &graphic_tx_buffer,
                count);

         
        if (send_result == HAL_BUSY)
        {
            return UI_BUSY;
        }

        if (send_result != HAL_OK)
        {
            return UI_ERROR;
        }

        ui_graphic_buffer_num = count;
        graphic_list_cursor = next_group_cursor;

        if (reached_end)
        {
            is_send_graphic_finish_flag = true;
            graphic_list_cursor = NULL;
        }

        return UI_BUSY;
    }

    if (is_send_char_finish_flag &&
        is_send_graphic_finish_flag)
    {
         
        char_list_cursor = NULL;
        graphic_list_cursor = NULL;

        is_send_char_finish_flag = false;
        is_send_graphic_finish_flag = false;

        return UI_OK;
    }

    return UI_ERROR;
}

/* Ui_Send */
void Ui_Send()
{
   
  uint32_t currentTick = HAL_GetTick();
  static uint32_t lastTick = 0;
  if (currentTick - lastTick < SEND_INTERVAL)
  {
    return;
  }
   
  static uint8_t is_initing_ui = 0;
  if (Init_Ui_Condition())
  {
    if (is_initing_ui == 0)
    {
      is_initing_ui = 1;
    }
  }
   
  static uint8_t init_times = 0;
  if (is_initing_ui == 1)
  {
    if (Ui_Send_Add() == UI_OK)
    {
      init_times++;
    }
    if (init_times >= PER_INIT_UI_TIMES)
    {
      is_initing_ui = 0;
      init_times = 0;
    }
  }
	else 
	{		
    Ui_Send_Normal();
	}
	lastTick = HAL_GetTick();
}

/* 接口说明 */
ui_status_e Enqueue_Ui_For_Sending(ui_info_t *ui_info)
{
  if (ui_info == NULL)
  {
    return UI_ERROR;
  }

  if (ui_info->sent_state == MESSAGE_SENT)
  {
    ui_info->updateTick = HAL_GetTick();
  }
  
  ui_info->sent_state = MESSAGE_NOT_SENT;

  return UI_OK;
}


static void Ui_Clear_Pending(void)
{
    memset(pending_graphic_ui, 0, sizeof(pending_graphic_ui));
    pending_graphic_num = 0;
    pending_character_ui = NULL;
}

static void Ui_Commit_Pending(void)
{
    for (uint8_t i = 0; i < pending_graphic_num; i++)
    {
        if (pending_graphic_ui[i] != NULL)
        {
            pending_graphic_ui[i]->sent_state = MESSAGE_SENT;
        }
    }

    if (pending_character_ui != NULL)
    {
        pending_character_ui->sent_state = MESSAGE_SENT;
    }

    Ui_Clear_Pending();
}

static void Ui_Rollback_Pending(void)
{
     
    for (uint8_t i = 0; i < pending_graphic_num; i++)
    {
        if (pending_graphic_ui[i] != NULL)
        {
            pending_graphic_ui[i]->sent_state = MESSAGE_NOT_SENT;
        }
    }

    if (pending_character_ui != NULL)
    {
        pending_character_ui->sent_state = MESSAGE_NOT_SENT;
    }

    Ui_Clear_Pending();
}

