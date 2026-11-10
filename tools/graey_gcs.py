"""Run on the laptop: python tools/graey_gcs.py --host GRAEY_WIFI_IP."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robotx_graey_2026.api.gui.gcs_transport import laptop_main

if __name__=='__main__':laptop_main()
