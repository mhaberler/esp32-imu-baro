// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.1
// LVGL version: 8.3.6
// Project name: lvgl_altimeter_UI

#include "../ui.h"

void ui_MENU_screen_init(void)
{
ui_MENU = lv_obj_create(NULL);
lv_obj_clear_flag( ui_MENU, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Top_Panel2 = lv_obj_create(ui_MENU);
lv_obj_set_width( ui_Top_Panel2, lv_pct(100));
lv_obj_set_height( ui_Top_Panel2, lv_pct(10));
lv_obj_set_align( ui_Top_Panel2, LV_ALIGN_TOP_MID );
lv_obj_clear_flag( ui_Top_Panel2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_Top_Panel2, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Top_Panel2, lv_color_hex(0xB8B8B8), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Top_Panel2, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label11 = lv_label_create(ui_Top_Panel2);
lv_obj_set_width( ui_Label11, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label11, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label11, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label11,"Settings");
lv_obj_set_style_text_color(ui_Label11, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label11, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label11, &ui_font_Font16, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Parameter_1 = lv_obj_create(ui_MENU);
lv_obj_set_width( ui_Parameter_1, 144);
lv_obj_set_height( ui_Parameter_1, 29);
lv_obj_set_x( ui_Parameter_1, lv_pct(0) );
lv_obj_set_y( ui_Parameter_1, lv_pct(10) );
lv_obj_clear_flag( ui_Parameter_1, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Label9 = lv_label_create(ui_Parameter_1);
lv_obj_set_width( ui_Label9, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label9, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label9, -32 );
lv_obj_set_y( ui_Label9, -1 );
lv_obj_set_align( ui_Label9, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label9,"BurnTime: ");
lv_obj_set_style_text_font(ui_Label9, &ui_font_Font12, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Parameter_2 = lv_obj_create(ui_MENU);
lv_obj_set_width( ui_Parameter_2, 144);
lv_obj_set_height( ui_Parameter_2, 29);
lv_obj_set_x( ui_Parameter_2, lv_pct(0) );
lv_obj_set_y( ui_Parameter_2, lv_pct(24) );
lv_obj_clear_flag( ui_Parameter_2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_text_font(ui_Parameter_2, &ui_font_Font16, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label5 = lv_label_create(ui_Parameter_2);
lv_obj_set_width( ui_Label5, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label5, LV_SIZE_CONTENT);   /// 1
lv_obj_set_y( ui_Label5, 0 );
lv_obj_set_x( ui_Label5, lv_pct(-15) );
lv_obj_set_align( ui_Label5, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label5,"heatCorrection:");
lv_obj_set_style_text_font(ui_Label5, &ui_font_Font12, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Parameter_3 = lv_obj_create(ui_MENU);
lv_obj_set_width( ui_Parameter_3, 144);
lv_obj_set_height( ui_Parameter_3, 29);
lv_obj_set_x( ui_Parameter_3, lv_pct(0) );
lv_obj_set_y( ui_Parameter_3, lv_pct(38) );
lv_obj_clear_flag( ui_Parameter_3, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_text_font(ui_Parameter_3, &ui_font_Font16, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label6 = lv_label_create(ui_Parameter_3);
lv_obj_set_width( ui_Label6, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label6, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label6, -17 );
lv_obj_set_y( ui_Label6, -1 );
lv_obj_set_align( ui_Label6, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label6,"negDynCorrect:");
lv_obj_set_style_text_font(ui_Label6, &ui_font_Font12, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Parameter_4 = lv_obj_create(ui_MENU);
lv_obj_set_width( ui_Parameter_4, 144);
lv_obj_set_height( ui_Parameter_4, 29);
lv_obj_set_x( ui_Parameter_4, lv_pct(0) );
lv_obj_set_y( ui_Parameter_4, lv_pct(52) );
lv_obj_clear_flag( ui_Parameter_4, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_text_font(ui_Parameter_4, &ui_font_Font12, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label8 = lv_label_create(ui_Parameter_4);
lv_obj_set_width( ui_Label8, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label8, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label8, -47 );
lv_obj_set_y( ui_Label8, 0 );
lv_obj_set_align( ui_Label8, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label8,"QNH:");
lv_obj_set_style_text_font(ui_Label8, &ui_font_Font16, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Parameter_5 = lv_obj_create(ui_MENU);
lv_obj_set_width( ui_Parameter_5, 144);
lv_obj_set_height( ui_Parameter_5, 29);
lv_obj_set_x( ui_Parameter_5, lv_pct(53) );
lv_obj_set_y( ui_Parameter_5, lv_pct(52) );
lv_obj_clear_flag( ui_Parameter_5, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_text_font(ui_Parameter_5, &ui_font_Font12, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label7 = lv_label_create(ui_Parameter_5);
lv_obj_set_width( ui_Label7, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label7, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label7, -47 );
lv_obj_set_y( ui_Label7, 0 );
lv_obj_set_align( ui_Label7, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label7,"xxx:");
lv_obj_set_style_text_font(ui_Label7, &ui_font_Font16, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Minus_Button = lv_btn_create(ui_MENU);
lv_obj_set_width( ui_Minus_Button, 53);
lv_obj_set_height( ui_Minus_Button, 47);
lv_obj_set_x( ui_Minus_Button, -129 );
lv_obj_set_y( ui_Minus_Button, 92 );
lv_obj_set_align( ui_Minus_Button, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Minus_Button, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_Minus_Button, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Minus_Button, lv_color_hex(0xDDD9A5), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Minus_Button, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Minus_Button, &ui_font_Font16, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label3 = lv_label_create(ui_Minus_Button);
lv_obj_set_width( ui_Label3, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label3, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label3, 1 );
lv_obj_set_y( ui_Label3, 2 );
lv_obj_set_align( ui_Label3, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label3,"-");
lv_obj_set_style_text_color(ui_Label3, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label3, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label3, &ui_font_Font24, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Plus_Button = lv_btn_create(ui_MENU);
lv_obj_set_width( ui_Plus_Button, 53);
lv_obj_set_height( ui_Plus_Button, 47);
lv_obj_set_x( ui_Plus_Button, 128 );
lv_obj_set_y( ui_Plus_Button, 92 );
lv_obj_set_align( ui_Plus_Button, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Plus_Button, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_Plus_Button, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Plus_Button, lv_color_hex(0xF0D11E), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Plus_Button, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Plus_Button, &ui_font_Font16, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label4 = lv_label_create(ui_Plus_Button);
lv_obj_set_width( ui_Label4, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label4, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label4, 2 );
lv_obj_set_y( ui_Label4, 0 );
lv_obj_set_align( ui_Label4, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label4,"+");
lv_obj_set_style_text_color(ui_Label4, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_text_opa(ui_Label4, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_text_font(ui_Label4, &ui_font_Font24, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_MENU, ui_event_MENU, LV_EVENT_ALL, NULL);

}