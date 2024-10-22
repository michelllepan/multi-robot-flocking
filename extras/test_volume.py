import time
import vlc

# Create a VLC instance
instance = vlc.Instance()

# Create a media player
player = instance.media_player_new()

# Load a media file (replace with your media file path)
media = instance.media_new('sounds/base.mp3')
player.set_media(media)

# Start playing
player.play()

# Set volume (0-100)
player.audio_set_volume(50)  # Sets volume to 50%

# Get current volume
current_volume = player.audio_get_volume()
print(f"Current volume: {current_volume}")

# Increase volume by 10%
player.audio_set_volume(min(current_volume + 10, 100))

# Decrease volume by 10%
player.audio_set_volume(max(current_volume - 10, 0))

player.play()

while True:
    current_volume = player.audio_get_volume()
    print(f"Current volume: {current_volume}")
    time.sleep(0.1)