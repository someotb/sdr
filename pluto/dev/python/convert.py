import subprocess
import sys


def pcm_to_mp3(pcm_path, mp3_path, sample_rate=44100, channels=2, bitrate="160k"):
    cmd = [
        "ffmpeg",
        "-y",  # перезаписать без вопросов
        "-f",
        "s16le",  # формат: signed 16-bit little-endian
        "-ar",
        str(sample_rate),  # частота дискретизации
        "-ac",
        str(channels),  # количество каналов
        "-i",
        pcm_path,  # входной файл
        "-b:a",
        bitrate,  # битрейт
        mp3_path,  # выходной файл
    ]
    subprocess.run(cmd, check=True)


def mp3_to_pcm(mp3_path, pcm_path, sample_rate=44100, channels=2):
    cmd = [
        "ffmpeg",
        "-y",
        "-i",
        mp3_path,
        "-f",
        "s16le",
        "-ar",
        str(sample_rate),
        "-ac",
        str(channels),
        pcm_path,
    ]
    subprocess.run(cmd, check=True)


print("""PCM->MP3: 0
MP3->PCM: 1""")
try:
    ans = int(input("Выберите режим: "))
except ValueError:
    print("Неверный ввод!")
    sys.exit(1)

if ans == 1:
    mp3_to_pcm(
        "../mp3/Flawed_Mangoes-Palindrome.mp3", "../pcm/Flawed_Mangoes-Palindrome.pcm"
    )
    print("MP3 -> PCM готово!")
elif ans == 0:
    pcm_to_mp3("../pcm/qam16.pcm", "../mp3/qam16.mp3")
    print("PCM -> MP3 готово!")
else:
    print("Выберите 0 или 1.")
