import numpy as np
import librosa
from pydub import AudioSegment

def mp3_to_pcm(src_path, dst_path, sample_rate=44100):
    audio_data, sr = librosa.load(src_path, sr=sample_rate, mono=False)
    if audio_data.ndim > 1:
        audio_data = audio_data.T.flatten()
    pcm_array = np.int16(audio_data * 32767)
    pcm_array.tofile(dst_path)

def pcm_to_mp3(pcm_path, output_path, channels=2, sample_rate=44100, bitrate="160k"):
    raw_audio = np.fromfile(pcm_path, dtype=np.int16)
    sound = AudioSegment(
        data=raw_audio.tobytes(),
        sample_width=2,
        frame_rate=sample_rate,
        channels=channels
    )
    sound.export(output_path, format="mp3", bitrate=bitrate)

print("""PCM->MP3: 0
MP3->PCM: 1""")
ans = int(input("Выберите режим: "))
if ans == 1:
    mp3_to_pcm("../1.mp3", "../1.pcm")
else:
    pcm_to_mp3("../2.pcm", "../2.mp3")
