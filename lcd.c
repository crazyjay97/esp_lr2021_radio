
WriteComm(0x11);     

Delay(480);                

WriteComm(0x36);     
WriteData(0x08);   

WriteComm(0x3A);     
WriteData(0x05);   

WriteComm(0xB2);     
WriteData(0x0C);   
WriteData(0x0C);   
WriteData(0x00);   
WriteData(0x33);   
WriteData(0x33);   

WriteComm(0xB7);     
WriteData(0x00);   

WriteComm(0xBB);     
WriteData(0x36);   

WriteComm(0xC2);     
WriteData(0x01);   

WriteComm(0xC3);     
WriteData(0x13);   //GVDD=4.8V 

WriteComm(0xC4);     
WriteData(0x20);   //VDV, 0x20:0v

WriteComm(0xC6);     
WriteData(0x0F);   //0x0F:60Hz        

WriteComm(0xD6);     
WriteData(0xA1);   	

WriteComm(0xD0);     
WriteData(0xA4);   
WriteData(0xA1);   

WriteComm(0xE0);     
WriteData(0xF0);   
WriteData(0x08);   
WriteData(0x0E);   
WriteData(0x09);   
WriteData(0x08);   
WriteData(0x04);   
WriteData(0x2F);   
WriteData(0x33);   
WriteData(0x45);   
WriteData(0x36);   
WriteData(0x13);   
WriteData(0x12);   
WriteData(0x2A);   
WriteData(0x2D);   

WriteComm(0xE1);     
WriteData(0xF0);   
WriteData(0x0E);   
WriteData(0x12);   
WriteData(0x0C);   
WriteData(0x0A);   
WriteData(0x15);   
WriteData(0x2E);   
WriteData(0x32);   
WriteData(0x44);   
WriteData(0x39);   
WriteData(0x17);   
WriteData(0x18);   
WriteData(0x2B);   
WriteData(0x2F);   

//WriteComm(0x21);     

//WriteComm(0x29); 


WriteComm(0x3A); //65k mode
WriteData(0x05);//05

WriteComm(0x29); //Display on
Delay(50);
WriteComm(0x2c);
Delay(20);