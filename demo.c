//19146378 Nguyen Le Hoang Quan
//19146376 Lai Tien Quang
//19146070 Nguyen Dinh Nhat
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
//gcc demo.c -o demo -l wiringPi -lm
// ./demo

#define CHANNEL     0
#define SPEED       8000000

#define DECODE_MODE_REGISTER            0x09
#define INTENSITY_REGISTER              0x0A
#define SCAN_LIMIT_REGISTER             0x0B
#define SHUTDOWN_REGISTER               0x0C
#define DISPLAYTEST_REGISTER            0x0F

#define MPU6050_ADDRESS                 0x68

#define SMPRT_DIV_REGEGISTER            0x19    // Sample Rate Divider (register 25)
#define CONFIG_REGISTER                 0x1A    // Configuration (register 26)
#define GYRO_CONFIG_REGISTER            0x1B    // Gyroscope Configuration (register 27)
#define ACCEL_CONFIG_REGISTER           0x1C    // Accelerometer Configuration (register 28)
#define INT_ENABLE_REGISTER             0x38    // Interrupt Enable (register 56)
#define PWR_MGMT_1_REGISTER             0x6B    // Power Management 1 (register 107)
#define ACCEL_XOUT_REGISTER             0x3B    // ACCEL_XOUT_H (register 59)
#define ACCEL_YOUT_REGISTER             0x3D    // ACCEL_YOUT_H (register 61)
#define ACCEL_ZOUT_REGISTER             0x3F    // ACCEL_ZOUT_H (register 63)
#define GYRO_XOUT_REGISTER              0x43    // GYRO_XOUT_H (register 67)
#define GYRO_YOUT_REGISTER              0x45    // GYRO_YOUT_H (register 69)
#define GYRO_ZOUT_REGISTER              0x47    // GYRO_ZOUT_H (register 71)
#define INT_ENABLE_REGISTER             0x38    // Interrupt enable (register 56)
#define INT_STATUS_REGISTER             0x3A    // Interrupt Status (register 58)

#define INT_DATA_READY_PIN              0       // Chân nhận tín hiệu ngắt từ MPU6050 là gpio0

/**************************************************************/

typedef enum
{
    start, running, over,
} game_state_t;

game_state_t gameState = start;

typedef enum
{
    huong_len = 0, huong_xuong = 2, huong_trai = 1, huong_phai = 3,
} huong_di_chuyen_t;

huong_di_chuyen_t huongRanDiChuyen = huong_phai, huongTuCamBien = huong_phai;

uint8_t mpu;
float ax, ay, az, roll, pitch;

uint8_t toaDoRan[64];
uint8_t giaTriHang[8];
uint8_t chieuDaiRan;
uint8_t toaDoMoi;

/*************************************************************************************/
/* Hàm đọc data từ 2 thanh ghi địa chỉ bắt đầu là register_address */
int16_t MPU6050_ReadRegister(uint8_t register_address)
{
    int16_t high_value, low_value, data;

    high_value = wiringPiI2CReadReg8(mpu, register_address);
    low_value = wiringPiI2CReadReg8(mpu, register_address + 1);
    data = (high_value << 8) | low_value;

    return data;
}

/* Hàm ngắt đọc dữ liệu khi dữ liệu sẵn sàng */
void interruptDataReady()
{
    // doc gia tri acc
    ax = MPU6050_ReadRegister(ACCEL_XOUT_REGISTER)/8192;
    ay = MPU6050_ReadRegister(ACCEL_YOUT_REGISTER)/8192;
    az = MPU6050_ReadRegister(ACCEL_ZOUT_REGISTER)/8192;
    
    roll = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2)))*180/M_PI;
    pitch = atan2(ax, sqrt(pow(ay, 2) + pow(az, 2)))*180/M_PI;

    if (pitch > 40 && roll >= -10 && roll <= 10)  
	{
		huongTuCamBien = huong_len;
    }
    else if (pitch < -40 && roll >= -10 && roll <= 10)  
	{
		huongTuCamBien = huong_xuong;
	}
    else if (roll > 40 && pitch >= -10 && pitch <= 10) 
	{
		huongTuCamBien = huong_trai;
	}
    else if (roll < -40 && pitch >= -10 && pitch <= 10) 
	{
		huongTuCamBien = huong_phai;
	}

    // xoa co ngat
    wiringPiI2CReadReg8(mpu, INT_STATUS_REGISTER);
}

/* Hàm cấu hình MPU6050 */
void MPU6050_Init()
{
    // sample rate = 100 Hz
    wiringPiI2CWriteReg8(mpu, SMPRT_DIV_REGEGISTER, 0x09);    
    // khong su dung nguon xung ngoai, DLFP < 44 Hz
    wiringPiI2CWriteReg8(mpu, CONFIG_REGISTER, 0x03);
    // gyro FS: +- 500 o/s
    wiringPiI2CWriteReg8(mpu, GYRO_CONFIG_REGISTER, 0x08);
    // acc FS: +- 2g
    wiringPiI2CWriteReg8(mpu, ACCEL_CONFIG_REGISTER, 0x00);
    // enable data ready interrupt
    wiringPiI2CWriteReg8(mpu, INT_ENABLE_REGISTER, 0x01);
    // power management
    wiringPiI2CWriteReg8(mpu, PWR_MGMT_1_REGISTER, 0x00);
    // khai bao su dung ngat data ready
    wiringPiI2CWriteReg8(mpu, INT_ENABLE_REGISTER, 0x01);
    pinMode(INT_DATA_READY_PIN, INPUT);
    wiringPiISR(INT_DATA_READY_PIN, INT_EDGE_RISING, &interruptDataReady);
}

/* Hàm gửi dữ liệu xuống MAX7219 */
void Send_Data(uint8_t address, uint8_t value)
{
    uint8_t buffer[2] = {address, value};

    wiringPiSPIDataRW(CHANNEL, buffer, 2);
}

/* Hàm cấu hình Max7219 */
void Max7219_Init()
{
    // decode mode:0x00
    Send_Data(DECODE_MODE_REGISTER, 0x00);
    // intensity
    Send_Data(INTENSITY_REGISTER, 0x08);
    // scan limit
    Send_Data(SCAN_LIMIT_REGISTER, 0x07);
    // no shutdown, display test off
    Send_Data(SHUTDOWN_REGISTER, 0x01);
    Send_Data(DISPLAYTEST_REGISTER, 0x00);
}

/*
* Khởi tạo vị trí rắn & mồi ban đầu
*/
void khoitao_Game()
{
    uint8_t temp, hang, cot;

    chieuDaiRan = 3;
    temp = 58;

    for (uint8_t i = 0; i < chieuDaiRan; i++)
    {
        toaDoRan[i] = temp--;
        hang = toaDoRan[i]/8;
        cot = toaDoRan[i]%8;
        giaTriHang[hang] |= (1 << cot);
    }

    toaDoMoi = 36;
    hang = toaDoMoi/8;
    cot = toaDoMoi%8;
    giaTriHang[hang] |= (1 << cot);
}

/*
* Xóa giá trị tọa độ rắn và mồi
*/
void reset_Game()
{
    for (uint8_t i = 0; i < 8; i++)
    {
        giaTriHang[i] = 0;
    }
}

/*
* In rắn & mồi lên max7219
*/
void inlen_ManHinh()
{
    for (uint8_t i = 1; i < 9; i++)
    {
        Send_Data(i, giaTriHang[i-1]);
    }
}

void xacdinh_HuongDiChuyen(huong_di_chuyen_t huongCamBien)
{
    // Xác định lại hướng rắn di chuyển dựa vào hướng trả về từ cảm biến
    if (((huongRanDiChuyen%2)-(huongCamBien%2)) != 0)
    {
        huongRanDiChuyen = huongCamBien;
    }
}

/*
* Xác định tọa độ rắn dựa vào hướng trả về từ cảm biến, không được ngược với hướng rắn di chuyển hiện tại
*/
void xacdinh_ToaDoRan( )
{
    uint8_t i, hang, cot; 

    // Xóa giá trị tọa độ rắn cũ
    for (i = 0; i < chieuDaiRan; i++)
    {
        hang = toaDoRan[i]/8;
        cot = toaDoRan[i]%8;
        giaTriHang[hang] &= ~(1 << cot);
    }
    
    // Xác định lại thân rắn mới
    for (i = chieuDaiRan-1; i >= 1; i--)
    {
        toaDoRan[i] = toaDoRan[i-1];
        hang = toaDoRan[i]/8;
        cot = toaDoRan[i]%8;
        giaTriHang[hang] |= (1 << cot);
    }

    // Xác định tọa độ đầu rắn mới theo hướng đã xác định
    switch (huongRanDiChuyen)
    {
        case huong_len:
            if (toaDoRan[i]/8 == 7)
            {
                toaDoRan[i] %= 8;
                hang = toaDoRan[i]/8;
                cot = toaDoRan[i]%8;
                giaTriHang[hang] |= (1 << cot);
            }
            else
            {
                toaDoRan[i] += 8;
                hang = toaDoRan[i]/8;
                cot = toaDoRan[i]%8;
                giaTriHang[hang] |= (1 << cot);
            }
            break;
        case huong_xuong:
            if (toaDoRan[i]/8 == 0)
            {
                toaDoRan[i] += 56;
                hang = toaDoRan[i]/8;
                cot = toaDoRan[i]%8;
                giaTriHang[hang] |= (1 << cot);
            }
            else
            {
                toaDoRan[i] -= 8;
                hang = toaDoRan[i]/8;
                cot = toaDoRan[i]%8;
                giaTriHang[hang] |= (1 << cot);
            }
            break;
        case huong_trai:
            if ((toaDoRan[i]%8) == 0)
            {
                toaDoRan[i] += 7;
                hang = toaDoRan[i]/8;
                cot = toaDoRan[i]%8;
                giaTriHang[hang] |= (1 << cot);
            }
            else
            {
                toaDoRan[i] -= 1;
                hang = toaDoRan[i]/8;
                cot = toaDoRan[i]%8;
                giaTriHang[hang] |= (1 << cot);
            }
            break;
        case huong_phai:
            if (toaDoRan[i]%8 == 7)
            {
                toaDoRan[i] -= 7;
                hang = toaDoRan[i]/8;
                cot = toaDoRan[i]%8;
                giaTriHang[hang] |= (1 << cot);
            }
            else
            {
                toaDoRan[i] += 1;
                hang = toaDoRan[i]/8;
                cot = toaDoRan[i]%8;
                giaTriHang[hang] |= (1 << cot);
            }
            break;
        default:
            break;
    }
}

/*
* Xác định tọa độ mồi của rắn
*/
void xacdinh_ToaDoMoi()
{
    uint8_t temp, hang, cot; 

    // xóa tọa độ mồi cũ
    hang = toaDoMoi/8;
    cot = toaDoMoi%8;
    giaTriHang[hang] &= ~(1 << cot);

    // tìm tọa độ mồi mới
    do 
    {
        temp = rand()%64;

        for (uint8_t i = 0; i < chieuDaiRan; i++)
        {
            if (temp == toaDoRan[i])
            {
                temp = toaDoMoi;
                break;
            }
        }
    } while (temp == toaDoMoi);

    toaDoMoi = temp;
    hang = toaDoMoi/8;
    cot = toaDoMoi%8;
    giaTriHang[hang] |= (1 << cot);
}

/*
* Kiểm tra rắn có ăn mồi hay không, trả về -> 1 rắn có ăn mồi
*/
uint8_t kiemtra_RanAnMoi()
{
    if (abs(toaDoMoi-toaDoRan[0]) == 8)
    {
        if ((huongRanDiChuyen == huong_len) || (huongRanDiChuyen == huong_xuong))
        {
            return 1;
        }
    }
    else if (abs(toaDoMoi-toaDoRan[0]) == 1)
    {
        if ((huongRanDiChuyen == huong_trai) || (huongRanDiChuyen == huong_phai))
        {
            return 1;
        }
    }

    return 0;
}

/*
* Tăng chiều dài rắn lên 1
*/
void tang_KichThuocRan()
{
    uint8_t i, hang, cot; 

    // xóa giá trị tọa độ rắn cũ
    for (uint8_t i = 0; i < chieuDaiRan; i++)
    {
        hang = toaDoRan[i]/8;
        cot = toaDoRan[i]%8;
        giaTriHang[hang] &= ~(1 << cot);
    }

    chieuDaiRan++;

    for (i = chieuDaiRan-1; i >= 1; i--)
    {
        toaDoRan[i] = toaDoRan[i-1];
        hang = toaDoRan[i]/8;
        cot = toaDoRan[i]%8;
        giaTriHang[hang] |= (1 << cot);
    }
    toaDoRan[i] = toaDoMoi;
    hang = toaDoRan[i]/8;
    cot = toaDoRan[i]%8;
    giaTriHang[hang] |= (1 << cot);
}

/*
* Kiểm tra rắn còn sống hay không, trả về 1 -> còn sống
*/
uint8_t kiemtra_RanConSong()
{
    for (uint8_t i = 1; i < chieuDaiRan; i++)
    {
        if (toaDoRan[0] == toaDoRan[i])
        {
            return 0;
        }
    }

    return 1;
}

/*
* Chạy chương trình chính, nhấn 'S'/'s' để bắt đầu game, sau khi thua nhấn lại để bắt đầu game mới không cần nạp lại code
*/
void chay_ChuongTrinh()
{
    uint32_t t_RanDiChuyen1, t_RanDiChuyen2, t_MoiXuatHien1, t_MoiXuatHien2;
    uint8_t diem = 0;
    char kyTu;

    switch (gameState)
    {
        case start:
            
            do
            {
                printf("Press 'S' to start a new game\n");
                scanf("%c", &kyTu);
                getchar();
            }
            while ((kyTu != 'S') && (kyTu != 's'));

            reset_Game();
            khoitao_Game();
            inlen_ManHinh();
            gameState = running;
            break;
        case running:
            t_RanDiChuyen1 = millis();
            t_MoiXuatHien1 = t_RanDiChuyen1;

            while (gameState == running)
            {
                t_RanDiChuyen2 = millis();
                if ((t_RanDiChuyen2 - t_RanDiChuyen1) >= 1000)
                {
                    // xác định hướng rắn
                    xacdinh_HuongDiChuyen(huongTuCamBien);
                    // kiểm tra rắn có ăn mồi hay không
                    if (kiemtra_RanAnMoi())
                    {
                        tang_KichThuocRan();
                        // tạo ra mồi mới
                        xacdinh_ToaDoMoi();
                        xacdinh_ToaDoRan();
                        diem++;
                        printf("Your scores: %d\n", diem);
                        inlen_ManHinh();
                        t_RanDiChuyen1 = millis();
                        t_MoiXuatHien1 = t_RanDiChuyen1;
                    }
                    else
                    {
                        xacdinh_ToaDoRan();
                        // kiểm tra rắn còn sống hay không
                        if (!kiemtra_RanConSong())
                        {
                            gameState = over;
                            break;
                        }
                        inlen_ManHinh();
                        t_RanDiChuyen1 = millis();
                    }
                }
        
                t_MoiXuatHien2 = millis();

                if ((t_MoiXuatHien2-t_MoiXuatHien1) >= 10000)
                {
                    //xacdinh_ToaDoMoi();
                    //inlen_ManHinh();
                    gameState = over;
                    break;
                    t_MoiXuatHien1 = millis();
                }
            }

            break;
        case over:
            printf("Game over!\n");
            gameState = start;
            diem = 0;
            break;
        default:
            break;
    }
}

int main()
{
    // set up wiringPi
    wiringPiSetup();
    // set up wiringPiSPI
    wiringPiSPISetup(CHANNEL, SPEED);
    // setup giao tiep I2C
    mpu = wiringPiI2CSetup(MPU6050_ADDRESS);
    // init MAX7219
    Max7219_Init();
    // init MPU6050
    MPU6050_Init();

    while (1)
    {
        chay_ChuongTrinh();
    }
}
