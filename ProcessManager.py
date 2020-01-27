import time
from Constants import Constants
import logging


class ProcessManager:
    def __init__(self, new_process_fn, update_fn,
                 disconnect_duration=Constants.DISCONNECT_DURATION,
                 restart_duration=Constants.RESTART_DURATION):
        """
        This class is in charge of restarting a process once it fails for a
        certain duration, the update method should be called regularly

        :param new_process_fn: callable, returns a new process
        :param update_fn: callable, returns a boolean whether the process is
        performing as expected
        """
        self.new_process_fn = new_process_fn
        self.update_fn = update_fn
        self.disconnect_duration = disconnect_duration
        self.restart_duration = restart_duration

        self.process = self.new_process_fn()
        self.logger = logging.getLogger(type(self.process).__name__ + 'Manager')
        self.logger.info('start process')
        self.process.start()


        self.last_connect_time = time.time()
        self.is_connected = False


    def update(self):
        if self.update_fn():
            self.last_connect_time = time.time()
            self.is_connected = True
        else:
            if time.time() - self.last_connect_time > self.disconnect_duration:
                self.is_connected = False
            if not self.is_connected and \
                    time.time() - self.last_connect_time > self.restart_duration:
                self.logger.error('process disconnected, try restart')
                if self.process.is_alive():
                    self.logger.error('process still alive, kill first')
                    self.process.terminate()

                self.process = self.new_process_fn()
                self.process.start()
                self.last_connect_time = time.time()
