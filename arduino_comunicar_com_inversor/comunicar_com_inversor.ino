#include <SPI.h>
#include <mcp2515.h>

volatile bool interrupt = false;
struct can_frame frame;
struct can_frame send_frame;

void irqHandler();
MCP2515 mcp2515(10);    // O construtor toma como entrada o pino do CS

void setup()
{
    Serial.begin(9600);
    SPI.begin();
    
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS);    // Coloque aqui a bitrate da sua rede
    mcp2515.setNormalMode();

    if(mcp2515.checkError()){
        Serial.println("mcp initialization failed, halting!");
        while(1);
    }
    
    attachInterrupt(2, irqHandler, FALLING);
}

void loop() 
{
    send_frame.can_id = 0x06B;
    send_frame.can_dlc = 1;
    send_frame.data[0] = 0x37;

    /* send out the message to the bus and
    tell other devices this is a standard frame from 0x00. */
    mcp2515.sendMessage(&send_frame);
    Serial.print("Id Enviado: ");
    Serial.print(send_frame.can_id);
    Serial.print("; ");
    Serial.print("Mensagem: 0x");
    Serial.print(send_frame.data[0], HEX);
    Serial.print("\n");

    if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
      Serial.print(frame.can_id, HEX); // print ID
      Serial.print(" "); 
      Serial.print(frame.can_dlc, HEX); // print DLC
      Serial.print(" ");
    
     for (int i = 0; i<frame.can_dlc; i++)  {  // print the data
        Serial.print(frame.data[i],HEX);
        Serial.print(" ");
    }

    Serial.println();      
  }

    //mcp2515.readMessage(MCP2515::RXB0, &frame);
    //print_can_data(frame);

    delay(1000);
}

void print_can_data(struct can_frame dataframe)
{
  Serial.print("Dado recebido; id: ");
  Serial.print(dataframe.can_id);
  Serial.print(", dlc: ");
  Serial.print(dataframe.can_dlc);

  Serial.print(", data = [ ");  

  for (int i = 0; i < dataframe.can_dlc; i++)
  {
    Serial.print(dataframe.data[i]);
    Serial.print(", ");
  }

  Serial.print("]\n");
}

void irqHandler() 
{
        uint8_t irq = mcp2515.getInterrupts();
        Serial.print("Mensagem Recebida:\n");

        if (irq & MCP2515::CANINTF_RX0IF) {
            if (mcp2515.readMessage(MCP2515::RXB0, &frame) == MCP2515::ERROR_OK) {
                print_can_data(frame);
            }
        }

        if (irq & MCP2515::CANINTF_RX1IF) {
            if (mcp2515.readMessage(MCP2515::RXB1, &frame) == MCP2515::ERROR_OK) {
                print_can_data(frame);
            }
        }
}
