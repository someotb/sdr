#  Software-Defined Radio(SDR)

Репозиторий со всеми версиями и лабораторными по практике (SDR -программно-определяемое радио)

### Установка библиотек и зависимостей:
```bash
bash install.sh
```

### Сборка:
```bash
cd pluto/dev
mkdir build
cd build
cmake ../
sudo make
```
### Запуск:

```bash
sudo ./main usb:0.0.0
```
Вместо `0.0.0` подставьте свое значение, которое можно узнать с помощью команды `SoapySDRUtil -f`

> Сайт с заметками и полезной информацией - [Yonote](https://sibsutis-rush.yonote.ru/share/93c85288-45ca-4532-8eab-4079899a5e1c/doc/lekcii-sdr-club-yadro-sibguti-kQcxQjrHno)

### Взлом Adalm PLuto

Время взлома! Откройте терминал (неважно, хост или виртуальная машина):

`ssh root@192.168.2.1`
Пароль по умолчанию - `analog`

Вы должны увидеть экран приветствия PlutoSDR. Теперь вы подключились по SSH к ARM-процессору на самом Pluto! Если у вас устройство Pluto с версией прошивки 0.31 или ниже, введите следующие команды:

```
fw_setenv attr_name compatible
fw_setenv attr_val ad9364
reboot
```
А для версии 0.32 и выше используйте:

```
fw_setenv compatible ad9364
reboot
```
Теперь вы можете настраивать частоты от **70 МГц** до **6 ГГц** , не говоря уже о частоте дискретизации до 56 МГц! Ура!

(до этого было от **325** МГц до **3.8** ГГц, из-за того что программно стояла AD9363)

**На случай ошибки:**  `ssh-keygen -f “/home/<user>/.ssh/known_hosts” -R “192.168.2.1”`

> [Оригинальная статья](https://nicoskin.notion.site/PlutoSDR-6e3f0880417f4927895cffa93ca2cf50)
