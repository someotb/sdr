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
Вместо 0.0.0 подставьте свое значение, которое можно узнать с помощью команды `SoapySDRUtil -f`

> Сайт с заметками и полезной информацией - [Yonote](https://sibsutis-rush.yonote.ru/share/93c85288-45ca-4532-8eab-4079899a5e1c/doc/lekcii-sdr-club-yadro-sibguti-kQcxQjrHno)
