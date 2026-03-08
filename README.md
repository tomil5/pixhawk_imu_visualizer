# Pixhawk IMU Reader

Программа для чтения данных IMU (акселерометр, гироскоп, магнитометр) с автопилота Pixhawk через MAVLink протокол и сохранения их в CSV файл.

## Описание

Это C++ приложение подключается к автопилоту Pixhawk через USB порт, получает данные с датчиков IMU в реальном времени и записывает их в CSV файл для последующего анализа.

**Получаемые данные:**
- Акселерометр (X, Y, Z) - ускорения в milli-G
- Гироскоп (X, Y, Z) - угловые скорости в milli-rad/s
- Магнитометр (X, Y, Z) - магнитное поле в milli-Gauss

## Требования

- **ОС:** Linux Mint, Ubuntu 20.04+ или другие Debian-based дистрибутивы
- **Компилятор:** GCC/G++ с поддержкой C++11
- **CMake:** версия 3.10 или выше
- **Оборудование:** Pixhawk автопилот с USB подключением

## Установка зависимостей

### Linux Mint / Ubuntu
```bash
# Обновляем систему
sudo apt update
sudo apt upgrade -y

# Устанавливаем необходимые пакеты
sudo apt install -y build-essential cmake git

# Добавляем пользователя в группу dialout для доступа к USB портам
sudo usermod -a -G dialout $USER

# ВАЖНО: Перелогиньтесь или перезагрузите компьютер после этой команды!
sudo reboot
```

## Установка и сборка
```bash
# 1. Клонируем репозиторий
git clone https://github.com/tomil5/pixhawk_imu_visualizer.git
cd pixhawk_imu_visualizer

# 2. Скачиваем MAVLink библиотеку (C заголовочные файлы)
git clone https://github.com/mavlink/c_library_v2.git

# 3. Создаём папку для сборки
mkdir build
cd build

# 4. Генерируем Makefile с помощью CMake
cmake ..

# 5. Компилируем проект
make
```

После успешной компиляции в папке `build/` появится исполняемый файл `pixhawk_imu_reader`.

## Запуск

### 1. Подключите Pixhawk к компьютеру через USB

### 2. Найдите порт устройства
```bash
# Проверьте какой порт используется
ls -l /dev/ttyACM*

# Обычно это /dev/ttyACM0
# Вывод: crw-rw---- 1 root dialout 166, 0 ... /dev/ttyACM0
```

### 3. Настройте порт в коде (если отличается)

Если у вас порт не `/dev/ttyACM0`, отредактируйте `src/imu_reader.cpp`:
```cpp
const char* PIXHAWK_PORT = "/dev/ttyACM0";  // Измените здесь
```

Затем пересоберите:
```bash
cd build
make
```

### 4. Запустите программу
```bash
cd build
./pixhawk_imu_reader
```

### Ожидаемый вывод
```
Connecting to Pixhawk...
Connected to /dev/ttyACM0
Listening for RAW_IMU messages...
[10] ACC: X=   -45 Y=   123 Z=  9810 | GYRO: X=    12 Y=   -34 Z=     5 | MAG: X=   234 Y=  -156 Z=   890
[20] ACC: X=   -42 Y=   125 Z=  9808 | GYRO: X=    10 Y=   -32 Z=     3 | MAG: X=   235 Y=  -154 Z=   892
...
```

### 5. Остановка программы

Нажмите **Ctrl+C** для корректного завершения.
```
^C
Stopping...
Program terminated. Total messages: 1523
Data saved to imu_log.csv
```

## Формат CSV файла

Данные сохраняются в файл `imu_log.csv` в следующем формате:
```csv
timestamp,AX_raw,AY_raw,AZ_raw,GX_raw,GY_raw,GZ_raw,MX_raw,MY_raw,MZ_raw
1738872345.123456,-45,123,9810,12,-34,5,234,-156,890
1738872345.223789,-42,125,9808,10,-32,3,235,-154,892
...
```

**Колонки:**
- `timestamp` - Unix timestamp в секундах с микросекундами
- `AX_raw, AY_raw, AZ_raw` - ускорения в milli-G (разделить на 1000 для получения G)
- `GX_raw, GY_raw, GZ_raw` - угловые скорости в milli-rad/s (разделить на 1000 для рад/с)
- `MX_raw, MY_raw, MZ_raw` - магнитное поле в milli-Gauss (разделить на 1000 для Gauss)

## Настройки

Вы можете изменить настройки в файле `src/imu_reader.cpp`:
```cpp
const char* PIXHAWK_PORT = "/dev/ttyACM0";  // USB порт
const int BAUDRATE = B115200;                // Скорость передачи данных
const char* CSV_FILE = "imu_log.csv";        // Имя файла для логов
bool LOG_TO_CSV = true;                      // Включить/выключить логирование
```

После изменений пересоберите проект:
```bash
cd build
make
```

## Решение проблем

### Ошибка: "Error opening port: /dev/ttyACM0"

**Причина:** Нет прав доступа к порту.

**Решение:**
```bash
# Проверьте что вы в группе dialout
groups

# Если нет dialout в списке:
sudo usermod -a -G dialout $USER
sudo reboot  # Обязательно перезагрузитесь!

# После перезагрузки проверьте снова
groups
```

### Устройство не найдено
```bash
# Проверьте подключённые USB устройства
ls -l /dev/ttyACM* /dev/ttyUSB*

# Проверьте в системных логах
dmesg | grep tty

# Попробуйте другой USB порт или кабель
```

### Нет данных / Timeout

**Возможные причины:**
- Pixhawk не загружен / в режиме bootloader
- Неправильный baudrate
- Проблемы с USB кабелем

**Решения:**
- Перезагрузите Pixhawk
- Попробуйте другой baudrate (57600):
```cpp
  const int BAUDRATE = B57600;
```
- Проверьте USB кабель (должен поддерживать передачу данных)

### Ошибки компиляции
```bash
# Проверьте что MAVLink библиотека скачана
ls c_library_v2/

# Если папки нет:
git clone https://github.com/mavlink/c_library_v2.git

# Очистите build и пересоберите
cd build
rm -rf *
cmake ..
make
```

## Структура проекта
```
pixhawk_imu_visualizer/
├── build/                  # Папка сборки (создаётся при компиляции)
│   └── pixhawk_imu_reader # Исполняемый файл
├── src/
│   └── pixhawk_imu_reader.cpp     # Исходный код программы
├── c_library_v2/          # MAVLink библиотека (клонируется отдельно)
├── CMakeLists.txt         # Конфигурация CMake
├── .gitignore
└── README.md
```