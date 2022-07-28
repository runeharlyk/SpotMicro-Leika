import logging
import asyncio
import json

from Movement import Movement
from IMU import IMU
from Gpio import Gpio

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler("debug.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

class Spot(object):
    """docstring for Spot."""

    def __init__(self, arg=None):
        super(Spot, self).__init__()
        logging.info("Booting up Leika")
        self.config = self.loadConfig("conf.json")

        self.threads = [
            Movement(),
            IMU(),
            Gpio(self.config)
        ]

    def loadConfig(self, uri):
        try:
            logging.info("Loading configuration")
            return json.load(open(uri))
        except FileNotFoundError as e:
            logging.warn("conf.json could not be located. Making one")
            with open("conf.json", "w") as outfile:
                json_conf = json.dumps({}, indent=4)
                outfile.write(json_conf)
        except json.decoder.JSONDecodeError as e:
            logging.warning("Failed to load config - Please check conf.json")
            logger.warning("Following exception was raised: {}".format(e))
        return False

    def startup(self):
        self.threads[2].button.when_pressed = self.threads[0].toogleMode
        logger.info("Starting threads:")
        for thread in self.threads:
            logger.info("Running thread: {thread_name}".format(thread_name=thread.name))
            thread.start()
        logger.info("Threads are started")

    def cleanup(self):
        logger.info("Terminating threads")
        for thread in self.threads:
            thread.terminate()
        logger.debug("Main loop finished (__main__")
        # Wait for actual termination (if needed)
        for thread in self.threads:
            thread.join()
        logger.info("All thread terminated - Closing Leika")

if __name__ == '__main__':
    Leika = Spot()
    Leika.startup()
    try:
        asyncio.get_event_loop().run_forever()
    except KeyboardInterrupt:
        logger.info("User interrupted program")
        Leika.cleanup()
