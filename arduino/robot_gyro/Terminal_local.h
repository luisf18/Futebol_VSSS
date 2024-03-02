#include "Terminal.h"


/***************************************/
/* ROBOT espnow                        */
/***************************************/

int terminal_espnow(String * in, String * out, int mode){
    int resp = (mode == TERMINAL_MODE_HELP ? TERMINAL_RETURN_HELP : TERMINAL_RETURN_NOT_FOUND);
    int r = terminal_execute_class(in, out, mode, "espnow");
    if (r == 2){
      if (mode == TERMINAL_MODE_HELP) *out += "\tMethods:\n";
      //TS_CALLER( "print", { T_RESULT() } )

      TS_CALLER( "print", {
        String txt;
        if( terminal_recursive_get_var( in, out, TERMINAL_MODE_GET, &txt ) ){ ESPNOWSerial.print(txt); }
        else{
          in->trim();
          int i;
          for(i=0;i<in->length() && !isspace(in->charAt(i));i++){  }
          ESPNOWSerial.print(in->substring(0,i));
          in->remove(0,i);
        }
        //resp = TERMINAL_RETURN_DONE;
      } )

      TS_CALLER( "println", {
        String txt;
        if( terminal_recursive_get_var( in, out, TERMINAL_MODE_GET, &txt ) ){ ESPNOWSerial.println(txt); }
        else{
          in->trim();
          int i;
          for(i=0;i<in->length() && !isspace(in->charAt(i));i++){  }
          ESPNOWSerial.println(in->substring(0,i));
          in->remove(0,i);
        }
        //resp = TERMINAL_RETURN_DONE;
      } )
      
      if (mode == TERMINAL_MODE_HELP) *out += "\tVariables:\n";
      
      if (mode == TERMINAL_MODE_HELP) *out += "\tDefines:\n";
      //resp = terminal_execute_define(in, out, mode, "OFF",         ESPNOW_device.OFF         ); COMMAND_LIST_CHECK(resp)
    }
    return resp;
}

/***************************************/
/* ROBOT pid                           */
/***************************************/


#define T_NO_TYPE 0
#define T_INT_t   1
#define T_FLOAR_t 2
#define T_PID_t   3

typedef struct Terminal_t{
  uint32_t type       = T_NO_TYPE;
  bool     is_pointer = false;
  uint32_t len        = 0;
  char     name[20];
  uint32_t val        = 0;
}Terminal_t;

Terminal_t T_buffer[50];
uint16_t T_buffer_len = 0;
void terminal_new( uint32_t type, const char *name, uint32_t val, uint32_t len, bool is_pointer ){
  T_buffer[T_buffer_len].type = type;
  strcpy(T_buffer[T_buffer_len].name, name);
  T_buffer[T_buffer_len].is_pointer = is_pointer;
  T_buffer[T_buffer_len].len = len;
  T_buffer[T_buffer_len].val = val;
  T_buffer_len++;
}

String teminal_get_next(String * in ){
  in->trim();
  int i;
  for(i=0;i<in->length() && !isspace(in->charAt(i));i++){}
  String result = in->substring(0,i);
  in->remove(0,i);
  return result;
}


int terminal_pid_metodos(String * in, String * out, int mode, Terminal_t T ){
    int resp = (mode == TERMINAL_MODE_HELP ? TERMINAL_RETURN_HELP : TERMINAL_RETURN_NOT_FOUND);
    
    if (mode == TERMINAL_MODE_HELP) *out += "\tVariables:\n";
    
    resp = terminal_execute_variable(in, out, mode, "kp",         &( ( (control_pid_t*) T.val )->kp        ) ); COMMAND_LIST_CHECK(resp)
    resp = terminal_execute_variable(in, out, mode, "ki",         &( ( (control_pid_t*) T.val )->ki        ) ); COMMAND_LIST_CHECK(resp)
    resp = terminal_execute_variable(in, out, mode, "kd",         &( ( (control_pid_t*) T.val )->kd        ) ); COMMAND_LIST_CHECK(resp)
    resp = terminal_execute_variable(in, out, mode, "last_erro",  &( ( (control_pid_t*) T.val )->last_erro ) ); COMMAND_LIST_CHECK(resp)
    resp = terminal_execute_variable(in, out, mode, "P",          &( ( (control_pid_t*) T.val )->P         ) ); COMMAND_LIST_CHECK(resp)
    resp = terminal_execute_variable(in, out, mode, "I",          &( ( (control_pid_t*) T.val )->I         ) ); COMMAND_LIST_CHECK(resp)
    resp = terminal_execute_variable(in, out, mode, "D",          &( ( (control_pid_t*) T.val )->D         ) ); COMMAND_LIST_CHECK(resp)
    resp = terminal_execute_variable(in, out, mode, "I_MAX",      &( ( (control_pid_t*) T.val )->I_MAX     ) ); COMMAND_LIST_CHECK(resp)

    return resp;
}

int terminal_pid(String * in, String * out, int mode){
    
    int resp = (mode == TERMINAL_MODE_HELP ? TERMINAL_RETURN_HELP : TERMINAL_RETURN_NOT_FOUND);

    if (mode == TERMINAL_MODE_HELP) *out += "\tVariables:\n";
    resp = terminal_execute_variable(in, out, mode, "loop",                   &loop_str                ); COMMAND_LIST_CHECK(resp)
    resp = terminal_execute_variable(in, out, mode, "loop_dt",                &loop_dt                 ); COMMAND_LIST_CHECK(resp)

    if (mode == TERMINAL_MODE_HELP) *out += "\tDefines:\n";
    resp = terminal_execute_define(in, out, mode, "CLEAR", "SAVE \"\" loop \"\" pid.auto 0 bot.stop"         ); COMMAND_LIST_CHECK(resp)

    int r = terminal_execute_class(in, out, mode, "pid");
    
    if (r == 2){

      if (mode == TERMINAL_MODE_HELP) *out += "\tMethods:\n";
      TS_CALLER( "imu", { T_RESULT(IMU.str()) } )
      TS_CALLER( "update", { IMU.update(); T_RESULT_VOID() } )
      resp = terminal_execute(in, out, mode, "run",     pid_angle  ); COMMAND_LIST_CHECK(resp)
      resp = terminal_execute(in, out, mode, "plot",    plot       ); COMMAND_LIST_CHECK(resp)
      resp = terminal_execute(in, out, mode, "run_w",   pid_w      ); COMMAND_LIST_CHECK(resp)
      resp = terminal_execute(in, out, mode, "plot_gz", plot_gz    ); COMMAND_LIST_CHECK(resp)
      resp = terminal_execute(in, out, mode, "test",    test       ); COMMAND_LIST_CHECK(resp)
      
      if (mode == TERMINAL_MODE_HELP) *out += "\tVariables:\n";
      resp = terminal_execute_variable(in, out, mode, "gz_raw",                 &IMU.g_raw.gyro.z        ); COMMAND_LIST_CHECK(resp)
      resp = terminal_execute_variable(in, out, mode, "g",                      &IMU.g.gyro.z            ); COMMAND_LIST_CHECK(resp)
      resp = terminal_execute_variable(in, out, mode, "a",                      &IMU.a.acceleration.x    ); COMMAND_LIST_CHECK(resp)
      resp = terminal_execute_variable(in, out, mode, "az_f",                   &IMU.ac_z_filter         ); COMMAND_LIST_CHECK(resp)
      resp = terminal_execute_variable(in, out, mode, "angle",                  &IMU.gyro_z_angle        ); COMMAND_LIST_CHECK(resp)
      resp = terminal_execute_variable(in, out, mode, "str",                    &pid_log                 ); COMMAND_LIST_CHECK(resp)
      resp = terminal_execute_variable(in, out, mode, "str_plot",               &pid_log_plot            ); COMMAND_LIST_CHECK(resp)
      resp = terminal_execute_variable(in, out, mode, "auto",                   &pid_auto                ); COMMAND_LIST_CHECK(resp)
      resp = terminal_execute_variable(in, out, mode, "auto_angle",             &pid_auto_angle          ); COMMAND_LIST_CHECK(resp)
      resp = terminal_execute_variable(in, out, mode, "auto_speed",             &pid_auto_speed          ); COMMAND_LIST_CHECK(resp)
      resp = terminal_execute_variable(in, out, mode, "test_result",            &test_result             ); COMMAND_LIST_CHECK(resp)
    }

    in->trim();

    // check in the buffer
    for( int i=0;i<T_buffer_len;i++ ){
      if (mode == TERMINAL_MODE_HELP){
        *out += "\t\nObject-> <" + String(T_buffer[i].type) + "> " +String(T_buffer[i].name) + "[" + String(T_buffer[i].len) + "] \n";
        terminal_pid_metodos(in, out, mode, T_buffer[i] );
      }else if( in->startsWith(T_buffer[i].name) ){
        in->remove(0,strlen(T_buffer[i].name));
        if ( mode == TERMINAL_MODE_SCAN && isspace(*(in->c_str())) ) return TERMINAL_RETURN_FOUND;
        if ( mode == TERMINAL_MODE_RUN || mode == TERMINAL_MODE_GET ){
          if ( *(in->c_str()) == '.' ){
            in->remove(0,1);
            return terminal_pid_metodos(in, out, mode, T_buffer[i] );
          }
        }
      }
    }

    TS_CALLER( "new", {
      in->trim();
      String type = teminal_get_next(in);
      if( type == "pid_t" ){
        String name = teminal_get_next(in);
        if( terminal_command( &name, out, TERMINAL_MODE_SCAN ) != TERMINAL_RETURN_FOUND ){
          terminal_new( T_PID_t, name.c_str(), (uint32_t) (new control_pid_t), 1, true );
        }
      }
    } )
    
    return resp;
}


/***************************************/
/* ROBOT (Motor + strategy)            */
/***************************************/

int terminal_bot(String * in, String * out, int mode ){
    int resp = (mode == TERMINAL_MODE_HELP ? TERMINAL_RETURN_HELP : TERMINAL_RETURN_NOT_FOUND);
    
    // espnow.x
    int r = terminal_execute_class(in, out, mode, "bot");
    if (r == 2){
      
      if (mode == TERMINAL_MODE_HELP) *out += "\tMethods:\n";
      TS_CALLER( "bip", {
        //T_VAR(uint8_t,n)
        //T_VAR(uint16_t,dt)
        //T_VAR(uint32_t,tone)
        //motor.bip(n,dt,tone);
        motor.bip(1,200,2000);
        T_RESULT_VOID()
      } )
      
      TS_CALLER( "move", {
        T_VAR(int,speed_0)
        T_VAR(int,speed_1)
        motor.move(speed_0,speed_1);
        T_RESULT_VOID()
      } )

      TS_CALLER( "stop", { motor.stop(); T_RESULT_VOID() } )
      TS_CALLER( "off", { motor.off(); T_RESULT_VOID() } )
      TS_CALLER( "read", { T_VAR(int,n) T_RESULT( motor.read(n) ) } )
      TS_CALLER( "voltage", { T_RESULT( voltage() ) } )
      
      if (mode == TERMINAL_MODE_HELP) *out += "\tVariables:\n";
      resp = terminal_execute_variable(in, out, mode, "voltage_alert",     &voltage_alert  ); COMMAND_LIST_CHECK(resp)

      if (mode == TERMINAL_MODE_HELP) *out += "\tDefines:\n";
      //resp = terminal_execute_define(in, out, mode, "OFF",         ESPNOW_device.OFF         ); COMMAND_LIST_CHECK(resp)
      
    }

    //if (mode == TERMINAL_MODE_HELP) *out += "Variables:\n";
    //resp = terminal_execute_variable(in, out, mode, "SAVE",  &terminal_save_cmd); COMMAND_LIST_CHECK(resp)
    //if (mode == TERMINAL_MODE_HELP) *out += "Functions:\n";
    //resp = terminal_execute(in, out, mode, "read",      digitalRead    ); COMMAND_LIST_CHECK(resp)
    
    return resp;
}