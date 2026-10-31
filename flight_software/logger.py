# ============================================================
#  TerraAvis — logger.py
# ============================================================
import logging
import os
from datetime import datetime
from config import LOG_DIR, LOG_LEVEL

os.makedirs(LOG_DIR, exist_ok=True)
_log_file = os.path.join(LOG_DIR, datetime.now().strftime("flight_%Y%m%d_%H%M%S.log"))

_level = getattr(logging, LOG_LEVEL.upper(), logging.DEBUG)

logging.basicConfig(
    level=_level,
    format="%(asctime)s [%(levelname)-8s] %(message)s",
    datefmt="%H:%M:%S",
    handlers=[
        logging.FileHandler(_log_file),
        logging.StreamHandler()
    ]
)

log = logging.getLogger("terraavis")
log.info(f"Log file: {_log_file}")