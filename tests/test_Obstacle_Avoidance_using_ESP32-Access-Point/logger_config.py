import logging
import sys

# Configure the logger
def setup_logging(level=logging.WARNING):
  if not logging.getLogger().hasHandlers():
    logging.basicConfig(
      level=level,  # Change to DEBUG for more details
      format="%(asctime)s - %(levelname)s - %(message)s",
      handlers=[
        logging.StreamHandler(sys.stdout),  # Explicitly use stdout
      ]
    )