"""V2 training templates. Synthetic smoke tests are not real-data validation."""
from pathlib import Path
import sys

# The B package owns the shared geometry, features and execution simulator.
WORKSPACE = Path(__file__).resolve().parents[3]
SWARM_ROOT = WORKSPACE / "swarm_cooperation" / "swarm_cooperation"
if str(SWARM_ROOT) not in sys.path:
    sys.path.insert(0, str(SWARM_ROOT))
