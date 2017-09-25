#include <avr/wdt.h>
#include <usbhub.h>

#define VendorID 0x0590
#define ProductID 0x005B
#define Max_RM_Index 240
#define Max_WM_Index 20
// #define ESP_Power 8

bool USB_RUN = false;
uint8_t addr, rcode;
String PLC_State = "PLC_Find_State";

const long CycleTime = 0;
unsigned long currentTime;
unsigned long previousTime = 0;
unsigned long timeout = 1000;

bool Monitor[6] = {1, 1, 1, 1, 1, 1};
uint16_t RM_Index = 0;
uint16_t WM_Index = 0;
uint16_t RM_Value[Max_RM_Index];
uint16_t WM_Value[Max_WM_Index];
String msg;
String Check_msg;

uint16_t RM_Address[Max_RM_Index] = {
    0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,
    14,  15,  16,  17,  18,  19,  20,  21,  22,  23,  24,  25,  26,  27,
    28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,

    100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113,
    114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127,
    128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139,

    200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213,
    214, 215, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227,
    228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239,

    300, 301, 302, 303, 304, 305, 306, 307, 308, 309, 310, 311, 312, 313,
    314, 315, 316, 317, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327,
    328, 329, 330, 331, 332, 333, 334, 335, 336, 337, 338, 339,

    400, 401, 402, 403, 404, 405, 406, 407, 408, 409, 410, 411, 412, 413,
    414, 415, 416, 417, 418, 419, 420, 421, 422, 423, 424, 425, 426, 427,
    428, 429, 430, 431, 432, 433, 434, 435, 436, 437, 438, 439,

    500, 501, 502, 503, 504, 505, 506, 507, 508, 509, 510, 511, 512, 513,
    514, 515, 516, 517, 518, 519, 520, 521, 522, 523, 524, 525, 526, 527,
    528, 529, 530, 531, 532, 533, 534, 535, 536, 537, 538, 539};
uint16_t WM_Address[Max_WM_Index] = {000, 000, 000, 000, 000, 000, 000,
                                     000, 000, 000, 000, 000, 000, 000,
                                     000, 000, 000, 000, 000, 000};
// bool Relay = 1;

USB Usb;
USB_DEVICE_DESCRIPTOR buf;

void setup() {
  wdt_disable();
  // pinMode(ESP_Power, OUTPUT);
  // digitalWrite(ESP_Power, HIGH);
  Serial.begin(115200);
  Serial3.begin(9600);
  wdt_enable(WDTO_8S);
  if (Usb.Init() == -1) {
    Serial.println("OSC did not start and Will be Reset!");
    asm volatile("jmp 0");
    while (1)
      ;
  }
  delay(200);
  Serial.println("///////////////////////////////////////////////");
  Serial.println("// Project Name     : Smart Machine          //");
  Serial.println("// Design By        : Mr.Grisada Chooprasert //");
  Serial.println("// Production By    : Mr.Puvanet             //");
  Serial.println("// Hardware Version : V16.C.20               //");
  Serial.println("// Software Version : V17.9.05               //");
  Serial.println("///////////////////////////////////////////////");
  Serial.println("");
}

void loop() {
  wdt_reset(); // make sure this gets called at least once every 8 seconds!
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= CycleTime) {
    previousTime = currentTime;
    do {
      Usb.Task();
      if (Usb.getUsbTaskState() == USB_STATE_RUNNING) {
        USB_RUN = true;

        if (PLC_State == "PLC_Find_State") {
          rcode = PLC_Find(VendorID, ProductID);
          if (rcode) {
            Serial.println("Omron PLC Not Found");
            while (1)
              ;
          } else {
            PLC_State = "PLC_RUN_Mode_Command_State";
            break;
          }
        }

        if (PLC_State == "PLC_RUN_Mode_Command_State") {
          rcode = PLC_RUN_Mode_Command();
          if (rcode == 0) {
            PLC_State = "PLC_RUN_Mode_Response_State";
            break;
          }
        }

        if (PLC_State == "PLC_RUN_Mode_Response_State") {
          rcode = PLC_RUN_Mode_Response();
          if (rcode == 0) {
            PLC_State = "PLC_EM_Read_Command_State";
            break;
          }
        }

        if (PLC_State == "PLC_EM_Read_Command_State") {
          rcode = PLC_EM_Read_Command(RM_Address[RM_Index]);
          if (rcode == 0) {
            PLC_State = "PLC_EM_Read_Response_State";
            break;
          }
        }

        if (PLC_State == "PLC_EM_Read_Response_State") {
          rcode = PLC_EM_Read_Response(&RM_Value[RM_Index]);
          if (rcode)
            PLC_State = "PLC_EM_Read_Command_State";
          else {
            RM_Index += 1;
            PLC_State = "PLC_EM_Read_Command_State";
          }
        }

        if (PLC_State == "PLC_EM_Read_Command_State" && RM_Index == 40) {
          if (Monitor[0]) {
            Serial.print(currentTime / 1000);
            Serial.print("|");
            msg = "FT001|" + String(RM_Value[0], HEX) + "|" +
                  String(RM_Value[1], HEX) + "|" + String(RM_Value[2], HEX) +
                  "|" + String(RM_Value[3], HEX) + "|" +
                  String(RM_Value[4], HEX) + "|" + String(RM_Value[5], HEX) +
                  "|" + String(RM_Value[6], HEX) + "|" +
                  String(RM_Value[7], HEX) + "|" + String(RM_Value[8], HEX) +
                  "|" + String(RM_Value[9], HEX) + "|" +
                  String(RM_Value[10], HEX) + "|" + String(RM_Value[11], HEX) +
                  "|" + String(RM_Value[12], HEX) + "|" +
                  String(RM_Value[13], HEX) + "|" + String(RM_Value[14], HEX) +
                  "|" + String(RM_Value[15], HEX) + "|" +
                  String(RM_Value[16], HEX) + "|" + String(RM_Value[17], HEX) +
                  "|" + String(RM_Value[18], HEX) + "|" +
                  String(RM_Value[19], HEX) + "|" + String(RM_Value[20], HEX) +
                  "|" + String(RM_Value[21], HEX) + "|" +
                  String(RM_Value[22], HEX) + "|" + String(RM_Value[23], HEX) +
                  "|" + String(RM_Value[24], HEX) + "|" +
                  String(RM_Value[25], HEX) + "|" + String(RM_Value[26], HEX) +
                  "|" + String(RM_Value[27], HEX) + "|" +
                  String(RM_Value[28], HEX) + "|" + String(RM_Value[29], HEX) +
                  "|" + String(RM_Value[30], HEX) + "|" +
                  String(RM_Value[31], HEX) + "|" + String(RM_Value[32], HEX) +
                  "|" + String(RM_Value[33], HEX) + "|" +
                  String(RM_Value[34], HEX) + "|" + String(RM_Value[35], HEX) +
                  "|" + String(RM_Value[36], HEX) + "|" +
                  String(RM_Value[37], HEX) + "|" + String(RM_Value[38], HEX) +
                  "|" + String(RM_Value[39], HEX) + "|";

            Serial3.print(msg);
            Serial3.print('\r');
            Serial.print(msg);
            Serial.print(" ");
            Check_msg = "";
            do {
              String line = Serial3.readStringUntil('\r');
              Serial.println(line);
              Check_msg = line;
            } while (Serial3.available() > 0);
          }
        }

        if (PLC_State == "PLC_EM_Read_Command_State" && RM_Index == 80) {
          if (Monitor[1]) {
            Serial.print(currentTime / 1000);
            Serial.print("|");
            msg = "FT002|" + String(RM_Value[40], HEX) + "|" +
                  String(RM_Value[41], HEX) + "|" + String(RM_Value[42], HEX) +
                  "|" + String(RM_Value[43], HEX) + "|" +
                  String(RM_Value[44], HEX) + "|" + String(RM_Value[45], HEX) +
                  "|" + String(RM_Value[46], HEX) + "|" +
                  String(RM_Value[47], HEX) + "|" + String(RM_Value[48], HEX) +
                  "|" + String(RM_Value[49], HEX) + "|" +
                  String(RM_Value[50], HEX) + "|" + String(RM_Value[51], HEX) +
                  "|" + String(RM_Value[52], HEX) + "|" +
                  String(RM_Value[53], HEX) + "|" + String(RM_Value[54], HEX) +
                  "|" + String(RM_Value[55], HEX) + "|" +
                  String(RM_Value[56], HEX) + "|" + String(RM_Value[57], HEX) +
                  "|" + String(RM_Value[58], HEX) + "|" +
                  String(RM_Value[59], HEX) + "|" + String(RM_Value[60], HEX) +
                  "|" + String(RM_Value[61], HEX) + "|" +
                  String(RM_Value[62], HEX) + "|" + String(RM_Value[63], HEX) +
                  "|" + String(RM_Value[64], HEX) + "|" +
                  String(RM_Value[65], HEX) + "|" + String(RM_Value[66], HEX) +
                  "|" + String(RM_Value[67], HEX) + "|" +
                  String(RM_Value[68], HEX) + "|" + String(RM_Value[69], HEX) +
                  "|" + String(RM_Value[70], HEX) + "|" +
                  String(RM_Value[71], HEX) + "|" + String(RM_Value[72], HEX) +
                  "|" + String(RM_Value[73], HEX) + "|" +
                  String(RM_Value[74], HEX) + "|" + String(RM_Value[75], HEX) +
                  "|" + String(RM_Value[76], HEX) + "|" +
                  String(RM_Value[77], HEX) + "|" + String(RM_Value[78], HEX) +
                  "|" + String(RM_Value[79], HEX) + "|";

            Serial3.print(msg);
            Serial3.print('\r');
            Serial.print(msg);
            Serial.print(" ");
            Check_msg = "";
            do {
              String line = Serial3.readStringUntil('\r');
              Serial.println(line);
              Check_msg = line;
            } while (Serial3.available() > 0);
          }
        }

        if (PLC_State == "PLC_EM_Read_Command_State" && RM_Index == 120) {
          if (Monitor[2]) {
            Serial.print(currentTime / 1000);
            Serial.print("|");
            msg =
                "FT003|" + String(RM_Value[80], HEX) + "|" +
                String(RM_Value[81], HEX) + "|" + String(RM_Value[82], HEX) +
                "|" + String(RM_Value[83], HEX) + "|" +
                String(RM_Value[84], HEX) + "|" + String(RM_Value[85], HEX) +
                "|" + String(RM_Value[86], HEX) + "|" +
                String(RM_Value[87], HEX) + "|" + String(RM_Value[88], HEX) +
                "|" + String(RM_Value[89], HEX) + "|" +
                String(RM_Value[90], HEX) + "|" + String(RM_Value[91], HEX) +
                "|" + String(RM_Value[92], HEX) + "|" +
                String(RM_Value[93], HEX) + "|" + String(RM_Value[94], HEX) +
                "|" + String(RM_Value[95], HEX) + "|" +
                String(RM_Value[96], HEX) + "|" + String(RM_Value[97], HEX) +
                "|" + String(RM_Value[98], HEX) + "|" +
                String(RM_Value[99], HEX) + "|" + String(RM_Value[100], HEX) +
                "|" + String(RM_Value[101], HEX) + "|" +
                String(RM_Value[102], HEX) + "|" + String(RM_Value[103], HEX) +
                "|" + String(RM_Value[104], HEX) + "|" +
                String(RM_Value[105], HEX) + "|" + String(RM_Value[106], HEX) +
                "|" + String(RM_Value[107], HEX) + "|" +
                String(RM_Value[108], HEX) + "|" + String(RM_Value[109], HEX) +
                "|" + String(RM_Value[110], HEX) + "|" +
                String(RM_Value[111], HEX) + "|" + String(RM_Value[112], HEX) +
                "|" + String(RM_Value[113], HEX) + "|" +
                String(RM_Value[114], HEX) + "|" + String(RM_Value[115], HEX) +
                "|" + String(RM_Value[116], HEX) + "|" +
                String(RM_Value[117], HEX) + "|" + String(RM_Value[118], HEX) +
                "|" + String(RM_Value[119], HEX) + "|";

            Serial3.print(msg);
            Serial3.print('\r');
            Serial.print(msg);
            Serial.print(" ");
            Check_msg = "";
            do {
              String line = Serial3.readStringUntil('\r');
              Serial.println(line);
              Check_msg = line;
            } while (Serial3.available() > 0);
          }
        }

        if (PLC_State == "PLC_EM_Read_Command_State" && RM_Index == 160) {
          if (Monitor[3]) {
            Serial.print(currentTime / 1000);
            Serial.print("|");
            msg =
                "FCT004|" + String(RM_Value[120], HEX) + "|" +
                String(RM_Value[121], HEX) + "|" + String(RM_Value[122], HEX) +
                "|" + String(RM_Value[123], HEX) + "|" +
                String(RM_Value[124], HEX) + "|" + String(RM_Value[125], HEX) +
                "|" + String(RM_Value[126], HEX) + "|" +
                String(RM_Value[127], HEX) + "|" + String(RM_Value[128], HEX) +
                "|" + String(RM_Value[129], HEX) + "|" +
                String(RM_Value[130], HEX) + "|" + String(RM_Value[131], HEX) +
                "|" + String(RM_Value[132], HEX) + "|" +
                String(RM_Value[133], HEX) + "|" + String(RM_Value[134], HEX) +
                "|" + String(RM_Value[135], HEX) + "|" +
                String(RM_Value[136], HEX) + "|" + String(RM_Value[137], HEX) +
                "|" + String(RM_Value[138], HEX) + "|" +
                String(RM_Value[139], HEX) + "|" + String(RM_Value[140], HEX) +
                "|" + String(RM_Value[141], HEX) + "|" +
                String(RM_Value[142], HEX) + "|" + String(RM_Value[143], HEX) +
                "|" + String(RM_Value[144], HEX) + "|" +
                String(RM_Value[145], HEX) + "|" + String(RM_Value[146], HEX) +
                "|" + String(RM_Value[147], HEX) + "|" +
                String(RM_Value[148], HEX) + "|" + String(RM_Value[149], HEX) +
                "|" + String(RM_Value[150], HEX) + "|" +
                String(RM_Value[151], HEX) + "|" + String(RM_Value[152], HEX) +
                "|" + String(RM_Value[153], HEX) + "|" +
                String(RM_Value[154], HEX) + "|" + String(RM_Value[155], HEX) +
                "|" + String(RM_Value[156], HEX) + "|" +
                String(RM_Value[157], HEX) + "|" + String(RM_Value[158], HEX) +
                "|" + String(RM_Value[159], HEX) + "|";

            Serial3.print(msg);
            Serial3.print('\r');
            Serial.print(msg);
            Serial.print(" ");
            Check_msg = "";
            do {
              String line = Serial3.readStringUntil('\r');
              Serial.println(line);
              Check_msg = line;
            } while (Serial3.available() > 0);
          }
        }

        if (PLC_State == "PLC_EM_Read_Command_State" && RM_Index == 200) {
          if (Monitor[4]) {
            Serial.print(currentTime / 1000);
            Serial.print("|");
            msg =
                "FCT005|" + String(RM_Value[160], HEX) + "|" +
                String(RM_Value[161], HEX) + "|" + String(RM_Value[162], HEX) +
                "|" + String(RM_Value[163], HEX) + "|" +
                String(RM_Value[164], HEX) + "|" + String(RM_Value[165], HEX) +
                "|" + String(RM_Value[166], HEX) + "|" +
                String(RM_Value[167], HEX) + "|" + String(RM_Value[168], HEX) +
                "|" + String(RM_Value[169], HEX) + "|" +
                String(RM_Value[170], HEX) + "|" + String(RM_Value[171], HEX) +
                "|" + String(RM_Value[172], HEX) + "|" +
                String(RM_Value[173], HEX) + "|" + String(RM_Value[174], HEX) +
                "|" + String(RM_Value[175], HEX) + "|" +
                String(RM_Value[176], HEX) + "|" + String(RM_Value[177], HEX) +
                "|" + String(RM_Value[178], HEX) + "|" +
                String(RM_Value[179], HEX) + "|" + String(RM_Value[180], HEX) +
                "|" + String(RM_Value[181], HEX) + "|" +
                String(RM_Value[182], HEX) + "|" + String(RM_Value[183], HEX) +
                "|" + String(RM_Value[184], HEX) + "|" +
                String(RM_Value[185], HEX) + "|" + String(RM_Value[186], HEX) +
                "|" + String(RM_Value[187], HEX) + "|" +
                String(RM_Value[188], HEX) + "|" + String(RM_Value[189], HEX) +
                "|" + String(RM_Value[190], HEX) + "|" +
                String(RM_Value[191], HEX) + "|" + String(RM_Value[192], HEX) +
                "|" + String(RM_Value[193], HEX) + "|" +
                String(RM_Value[194], HEX) + "|" + String(RM_Value[195], HEX) +
                "|" + String(RM_Value[196], HEX) + "|" +
                String(RM_Value[197], HEX) + "|" + String(RM_Value[198], HEX) +
                "|" + String(RM_Value[199], HEX) + "|";

            Serial3.print(msg);
            Serial3.print('\r');
            Serial.print(msg);
            Serial.print(" ");
            Check_msg = "";
            do {
              String line = Serial3.readStringUntil('\r');
              Serial.println(line);
              Check_msg = line;
            } while (Serial3.available() > 0);
          }
        }

        if (PLC_State == "PLC_EM_Read_Command_State" && RM_Index == 240) {
          if (Monitor[5]) {
            Serial.print(currentTime / 1000);
            Serial.print("|");
            msg =
                "FCT006|" + String(RM_Value[200], HEX) + "|" +
                String(RM_Value[201], HEX) + "|" + String(RM_Value[202], HEX) +
                "|" + String(RM_Value[203], HEX) + "|" +
                String(RM_Value[204], HEX) + "|" + String(RM_Value[205], HEX) +
                "|" + String(RM_Value[206], HEX) + "|" +
                String(RM_Value[207], HEX) + "|" + String(RM_Value[208], HEX) +
                "|" + String(RM_Value[209], HEX) + "|" +
                String(RM_Value[210], HEX) + "|" + String(RM_Value[211], HEX) +
                "|" + String(RM_Value[212], HEX) + "|" +
                String(RM_Value[213], HEX) + "|" + String(RM_Value[214], HEX) +
                "|" + String(RM_Value[215], HEX) + "|" +
                String(RM_Value[216], HEX) + "|" + String(RM_Value[217], HEX) +
                "|" + String(RM_Value[218], HEX) + "|" +
                String(RM_Value[219], HEX) + "|" + String(RM_Value[220], HEX) +
                "|" + String(RM_Value[221], HEX) + "|" +
                String(RM_Value[222], HEX) + "|" + String(RM_Value[223], HEX) +
                "|" + String(RM_Value[224], HEX) + "|" +
                String(RM_Value[225], HEX) + "|" + String(RM_Value[226], HEX) +
                "|" + String(RM_Value[227], HEX) + "|" +
                String(RM_Value[228], HEX) + "|" + String(RM_Value[229], HEX) +
                "|" + String(RM_Value[230], HEX) + "|" +
                String(RM_Value[231], HEX) + "|" + String(RM_Value[232], HEX) +
                "|" + String(RM_Value[233], HEX) + "|" +
                String(RM_Value[234], HEX) + "|" + String(RM_Value[235], HEX) +
                "|" + String(RM_Value[236], HEX) + "|" +
                String(RM_Value[237], HEX) + "|" + String(RM_Value[238], HEX) +
                "|" + String(RM_Value[239], HEX) + "|";

            Serial3.print(msg);
            Serial3.print('\r');
            Serial.print(msg);
            Serial.print(" ");
            Check_msg = "";
            do {
              String line = Serial3.readStringUntil('\r');
              Serial.println(line);
              Check_msg = line;
            } while (Serial3.available() > 0);
          }
        }

        // xxxxxxxxxxxxxx

        if (RM_Index == Max_RM_Index) {
          RM_Index = 0;
          Serial.println("");
        }

      } else {
        if (USB_RUN) {
          Serial.println("USB Not RUN and Will be Reset");
          delay(1000);
          asm volatile("jmp 0");
        }
      } // End USB Run loop
    } while (RM_Index == Max_RM_Index);
  } // End scan time loop
}

void GetAddresses(UsbDevice *pdev) {
  UsbDeviceAddress adr;
  adr.devAddress = pdev->address.devAddress;
  addr = adr.devAddress;
}

uint8_t PLC_Find(uint16_t Vendor_ID, uint16_t Product_ID) {
  uint8_t rcode;
  Usb.ForEachUsbDevice(&GetAddresses);
  rcode =
      Usb.getDevDescr(addr, 0, sizeof(USB_DEVICE_DESCRIPTOR), (uint8_t *)&buf);
  if (rcode) {
    return (rcode);
  } else {
    if (buf.idVendor == Vendor_ID && buf.idProduct == Product_ID) {
      rcode = Usb.setConf(addr, 0, buf.bNumConfigurations);
      return (rcode);
    } else {
      return (USB_STATE_ERROR);
    }
  }
  return (USB_STATE_ERROR);
}

uint8_t PLC_RUN_Mode_Command() {
  uint8_t rcode;
  uint8_t msg[20] = {0xAB, 0x00, 0x11, 0x80, 0x00, 0x02,
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  msg[12] = random(1, 255);
  msg[13] = 0x04;
  msg[14] = 0x01;
  msg[15] = 0xFF;
  msg[16] = 0xFF;
  msg[17] = 0x04;
  uint16_t sumcheck = 0;
  for (uint16_t i = 0; i < 18; i++) {
    sumcheck += msg[i];
  }

  msg[18] = ((sumcheck >> 8) & 0xFF);
  msg[19] = (sumcheck & 0xFF);

  rcode = PLC_TxD(sizeof(msg), msg);
  if (rcode)
    return rcode;
  return (rcode);
}

uint8_t PLC_RUN_Mode_Response() {
  uint8_t rcode;
  uint8_t buf[64];
  uint16_t rcvd = 0;

  rcode = PLC_RxD(&rcvd, buf);
  //  RecPrintHEX(rcvd,buf);

  if (rcode)
    return rcode;

  if (rcvd == 19) {
    uint8_t val1 = buf[rcvd - 1];
    uint8_t val2 = buf[rcvd - 2];
    uint16_t sum1 = val2 << 8 | val1;
    uint16_t sum2 = 0;
    for (uint16_t i = 0; i < 17; i++) {
      sum2 += buf[i];
    }

    if (sum1 == sum2) {
      uint8_t revc[17] = {0xAB, 0x0, 0x10, 0xC0, 0x0, 0x2, 0x0, 0x0, 0xFB,
                          0x0,  0x0, 0x0,  0x30, 0x4, 0x1, 0x0, 0x0};
      for (uint16_t i = 0; i < 17; i++) {
        if (revc[i] != buf[i] && i != 12)
          return (2);
      }
      return (0);
    } else {
      return (1);
    }
  }
  return (rcode);
}

uint8_t PLC_EM_Read_Command(uint16_t D_number) {
  uint8_t rcode;
  uint8_t msg[25] = {0xAB, 0x00, 0x16, 0x80, 0x00, 0x2,
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  msg[12] = random(1, 255);
  msg[13] = 0x01;
  msg[14] = 0x04;
  msg[15] = 0x07;
  msg[16] = 0x00;
  msg[17] = 0x00;
  msg[18] = 0x00;
  msg[19] = 0xA0; // 0x82 0xA0
  // ///////////////////////////////////////////////////////////////////////
  msg[20] = (D_number >> 8);
  msg[21] = (D_number & 0xFF);
  msg[22] = 0x00;

  uint16_t sumcheck = 0;
  for (uint16_t i = 0; i < 23; i++) {
    sumcheck += msg[i];
  }

  msg[23] = ((sumcheck >> 8) & 0xFF);
  msg[24] = (sumcheck & 0xFF);

  rcode = PLC_TxD(sizeof(msg), msg);
  return (rcode);
}

uint8_t PLC_EM_Read_Response(uint16_t *D_value) { // 342-E1-15 EM=20
  uint8_t rcode;
  uint8_t buf[64];
  uint16_t rcvd = 0;

  //  rcode = PLC_RxD(&rcvd, buf);
  do {
    rcode = PLC_RxD(&rcvd, buf);
  } while (rcvd == 0);
  //  RecPrintHEX(rcvd, buf);

  if (rcode)
    return rcode;

  if (rcvd == 24) {
    uint8_t val1 = buf[rcvd - 1];
    uint8_t val2 = buf[rcvd - 2];
    uint16_t sum1 = val2 << 8 | val1;
    uint16_t sum2 = 0;
    for (uint16_t i = 0; i < 22; i++) {
      sum2 += buf[i];
    }

    if (sum1 == sum2) {
      uint8_t revc[18] = {0xAB, 0x0, 0x15, 0xC0, 0x0, 0x2, 0x0, 0x0, 0xFB,
                          0x0,  0x0, 0x0,  0x9C, 0x1, 0x4, 0x0, 0x0, 0x7};
      for (uint16_t i = 0; i < 18; i++) {
        if (revc[i] != buf[i] && i != 12)
          return (2);
      }
      uint8_t val1 = buf[rcvd - 3];
      uint8_t val2 = buf[rcvd - 4];
      uint16_t value = val2 << 8 | val1;
      *D_value = value;
      return (0);
    } else {
      return (1);
    }
  }
  return (1);
}

uint8_t PLC_EM_Write_Command(uint16_t D_number, uint16_t D_value) {
  uint8_t rcode;
  uint8_t msg[25] = {0xAB, 0x00, 0x16, 0x80, 0x00, 0x2,
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  msg[12] = random(1, 255);
  msg[13] = 0x01;
  msg[14] = 0x02;
  msg[15] = 0xA0; // 0x82 0xA0
  // /////////////////////////////////////////////////////////////////////////
  msg[16] = (D_number >> 8);
  msg[17] = (D_number & 0xFF);
  msg[18] = 0x00;
  msg[19] = 0x00;
  msg[20] = 0x01;
  msg[21] = (D_value >> 8);
  msg[22] = (D_value & 0xFF);

  uint16_t sumcheck = 0;
  for (uint16_t i = 0; i < 23; i++) {
    sumcheck += msg[i];
  }

  msg[23] = ((sumcheck >> 8) & 0xFF);
  msg[24] = (sumcheck & 0xFF);

  rcode = PLC_TxD(sizeof(msg), msg);

  return (rcode);
}

uint8_t PLC_EM_Write_Response() {
  uint8_t rcode;

  uint8_t buf[64];
  uint16_t rcvd = 0;

  //  rcode = PLC_RxD(&rcvd, buf);
  do {
    rcode = PLC_RxD(&rcvd, buf);
  } while (rcvd == 0);
  //  RecPrintHEX(rcvd, buf);

  if (rcode)
    return rcode;

  if (rcvd == 19) {
    uint8_t val1 = buf[rcvd - 1];
    uint8_t val2 = buf[rcvd - 2];
    uint16_t sum1 = val2 << 8 | val1;
    uint16_t sum2 = 0;
    for (uint16_t i = 0; i < 17; i++) {
      sum2 += buf[i];
    }

    if (sum1 == sum2) {
      uint8_t revc[17] = {0xAB, 0x0, 0x10, 0xC0, 0x0, 0x2, 0x0, 0x0, 0xFB,
                          0x0,  0x0, 0x0,  0x6F, 0x1, 0x2, 0x0, 0x0};
      for (uint16_t i = 0; i < 17; i++) {
        if (revc[i] != buf[i] && i != 12)
          return (3);
      }
      return (0);
    } else {
      return (2);
    }
  }
  return (1);
}

void RecPrintHEX(uint16_t nbytes, uint8_t *data) {
  if (nbytes == 0)
    return;
  for (uint16_t i = 0; i < nbytes; i++) {
    Serial.print(" ,0x");
    Serial.print(data[i], HEX);
  }
  Serial.println("");
}

uint8_t PLC_TxD(uint16_t nbytes, uint8_t *data) {
  Usb.bytesWr(rSNDFIFO, nbytes, data);
  Usb.regWr(rSNDBC, nbytes);
  Usb.regWr(rHXFR, 0x21);
  while (!(Usb.regRd(rHIRQ) & bmHXFRDNIRQ))
    ;                            // Wait for the completion IRQ
  Usb.regWr(rHIRQ, bmHXFRDNIRQ); // Clear IRQ
  return (0);
}

uint8_t PLC_RxD(uint16_t *pktsize, uint8_t *data) {
  //  unsigned long timeout_start = millis() + USB_XFER_TIMEOUT;
  unsigned long timeout_start = millis() + timeout;

  while ((long)(millis() - timeout_start) < 0L) {
    Usb.regWr(rHXFR, 0x2);
    if ((Usb.regRd(rHIRQ) & bmRCVDAVIRQ) == bmRCVDAVIRQ) {
      uint16_t buff_pktsize = Usb.regRd(rRCVBC);
      *pktsize = buff_pktsize;
      data = Usb.bytesRd(rRCVFIFO, buff_pktsize, data);
      Usb.regWr(rHIRQ, bmRCVDAVIRQ); // Clear the IRQ & free the buffer
      return (0);
    }
  }
  return (1);
}

/*
  Note
  1 byte -> [0-255] or [0x00-0xFF]
  byte
  uint8_t
  unsigned char

  2 bytes -> [0-65535] or [0x0000-0xFFFF]
  word
  uint16_t
  unsigned short

  4 bytes -> [0-4294967295] or [0x00000000-0xFFFFFFFF]
  unsigned long
  uint32_t

  asm volatile("jmp 0"); // goes to the beginning of the application section
  asm volatile ("jmp 0x1F000"); // goes to the beginning of the bootloader
  section
*/

//                                      0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 ,
//                                      10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
//                                      20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
//                                      30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
//                                      40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
//                                      50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
//
//                                      60 , 61  , 62 , 63 , 64 , 65 , 66 , 67 ,
//                                      68 , 69 , 70 , 71 , 72 , 73 , 74 , 75 ,
//                                      76 , 77 , 78 , 79 ,
//                                      80 , 81  , 82 , 83 , 84 , 85 , 86 , 87 ,
//                                      88 , 89 , 90 , 91 , 92 , 93 , 94 , 95 ,
//                                      96 , 97 , 98 , 99 ,
//                                      100, 101 , 102, 103, 104, 105, 106, 107,
//                                      108, 109, 110, 111, 112, 113, 114, 115,
//                                      116, 117, 118, 119,
//
//                                      120, 121 , 122, 123, 124, 125, 126, 127,
//                                      128, 129, 130, 131, 132, 133, 134, 135,
//                                      136, 137, 138, 139,
//                                      140, 141 , 142, 143, 144, 145, 146, 147,
//                                      148, 149, 150, 151, 152, 153, 154, 155,
//                                      156, 157, 158, 159,
//                                      160, 161 , 162, 163, 164, 165, 166, 167,
//                                      168, 169, 170, 171, 172, 173, 174, 175,
//                                      176, 177, 178, 179,
//
//                                      180, 181 , 182, 183, 184, 185, 186, 187,
//                                      188, 189, 190, 191, 192, 193, 194, 195,
//                                      196, 197, 198, 199,
//                                      200, 201 , 202, 203, 204, 205, 206, 207,
//                                      208, 209, 210, 211, 212, 213, 214, 215,
//                                      216, 217, 218, 219,
//                                      220, 221 , 222, 223, 224, 225, 226, 227,
//                                      228, 229, 230, 231, 232, 233, 234, 235,
//                                      236, 237, 238, 239,
//
//                                      240, 241 , 242, 243, 244, 245, 246, 247,
//                                      248, 249, 250, 251, 252, 253, 254, 255,
//                                      256, 257, 258, 259,
//                                      260, 261 , 262, 263, 264, 265, 266, 267,
//                                      268, 269, 270, 271, 272, 273, 274, 275,
//                                      276, 277, 278, 279,
//                                      280, 281 , 282, 283, 284, 285, 286, 287,
//                                      288, 289, 290, 291, 292, 293, 294, 295,
//                                      296, 297, 298, 299

// (String) RM_Value[0]
