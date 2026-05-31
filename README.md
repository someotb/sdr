#  Software-Defined Radio(SDR)

Репозиторий со всеми версиями и лабораторными по практике (SDR -программно-определяемое радио)

### Установка библиотек и зависимостей:
```bash
bash install.sh
```

После клона также нужно подтянуть сабмодули:
```bash
git submodule update --init --recursive
```

Также нужно отдельно собрать все библиотеки для работы с SDR.
Собирать нужно в таком порядке:
```bash
1) libiio - Сперва ее, т.к от нее собираются все остальные
2) libad9361-iio
3) SoapySDR
4) SoapyPlutoSDR
```

После сборки и установки библиотеки в систему, нужно обновлять зависимости:
```bash
sudo ldconfig
```

### Сборка:
```bash
mkdir build
cd build
cmake ..
cmake --build . -j
```
### Запуск:

```bash
./main
```

> Сайт с заметками и полезной информацией - [Yonote](https://sibsutis-rush.yonote.ru/share/93c85288-45ca-4532-8eab-4079899a5e1c/doc/lekcii-sdr-club-yadro-sibguti-kQcxQjrHno)

### Взлом Adalm PLuto

[Оригинальная статья](https://nicoskin.notion.site/PlutoSDR-6e3f0880417f4927895cffa93ca2cf50)
