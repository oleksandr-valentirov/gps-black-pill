import argparse
import serial
import ctypes
import sys
from serial.tools import list_ports

# Створюємо функцію для парсингу аргументів командного рядка
def parse_args():
    parser = argparse.ArgumentParser(description='Парсинг даних з COM-порту.')
    parser.add_argument('port', type=str, help='COM-порт для підключення')
    parser.add_argument('-l', '--list', action='store_true', help='Вивести список доступних COM-портів')
    return parser.parse_args()

# Функція для виведення списку доступних COM-портів
def list_ports():
    ports = list_ports.comports()
    if not ports:
        print("Немає доступних COM-портів.")
    else:
        for port in ports:
            print(port.device)

# Визначення структури заголовка
class Header(ctypes.LittleEndianStructure):
    _fields_ = [
        ("sync1", ctypes.c_uint8),
        ("sync2", ctypes.c_uint8),
        ("checksum", ctypes.c_uint16),
        ("classID", ctypes.c_uint8),
        ("length", ctypes.c_uint8)
    ]

# Визначення структури даних
class Data(ctypes.LittleEndianStructure):
    _fields_ = [
        ("lon", ctypes.c_int32),
        ("lat", ctypes.c_int32),
        ("gSpeed", ctypes.c_int32),
        ("heading", ctypes.c_int32),
        ("velN", ctypes.c_int32),
        ("velE", ctypes.c_int32),
        ("nano", ctypes.c_int32),
        ("year", ctypes.c_uint16),
        ("month", ctypes.c_uint8),
        ("day", ctypes.c_uint8),
        ("hour", ctypes.c_uint8),
        ("minute", ctypes.c_uint8),
        ("sec", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8)
    ]

def process_data(ser):
    # Розмір структур в байтах
    header_size = ctypes.sizeof(Header)
    data_size = ctypes.sizeof(Data)
    
    while True:
        # Очікуємо на синхронізацію
        sync1 = ser.read(1)
        if sync1 == b'\x47':  # 71 в десятковій системі
            sync2 = ser.read(1)
            if sync2 == b'\x49':  # 73 в десятковій системі
                # Читаємо заголовок
                header_data = ser.read(header_size - 2)
                if len(header_data) != header_size - 2:
                    print("Помилка: недостатньо байтів для заголовка.")
                    continue
                
                # Розпаковуємо заголовок
                header = Header.from_buffer_copy(bytes(sync1) + bytes(sync2) + header_data)
                
                # Читаємо дані
                data = ser.read(data_size)
                if len(data) != data_size:
                    print("Помилка: недостатньо байтів для даних.")
                    continue
                
                # Розпаковуємо дані
                data_struct = Data.from_buffer_copy(data)
                
                # Виводимо результат
                print(f"lon: {data_struct.lon / 1e7}")
                print(f"lat: {data_struct.lat / 1e7}")
                print(f"{data_struct.day}/{data_struct.month}/{data_struct.year} {data_struct.hour}:{data_struct.minute}:{data_struct.sec}")
                print("###########################################")

def main():
    args = parse_args()
    
    if args.list:
        list_ports()
    else:
        # Ініціалізація серійного з'єднання
        ser = serial.Serial(args.port, 9600, timeout=3)
        try:
            process_data(ser)
        except Exception as e:
            print(e)
        finally:
            ser.close()

if __name__ == "__main__":
    main()
