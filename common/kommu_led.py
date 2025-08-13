import subprocess

COLORS = {
  "WHITE": "FFFFFF",
  "GREEN": "178644",
  "ORANGE": "DA6F25",
  "RED": "C92231"
}

def set(color=None, mode=None, rate=None):
  mode = mode or "solid"
  if rate and not mode:
    mode = "blink"
  args = ["python", "/usr/kommu/ws2812.py", mode]
  if mode in ("solid", "blink", "run"):
    if color:
      args += ["--a-color", color, "--b-color", color]
    if mode == "blink" and rate:
      args += ["--rate", rate]
  subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
  #print(f"LED set to: mode={mode}, color={color}, rate={rate}")
