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
        self.update_rate = 0
        self.is_connected = False
        self.last_update_time = time.time()


    def update(self):
        if self.update_fn():
            rate = 1 / (time.time() - self.last_connect_time)
            self.update_rate = self.update_rate * Constants.MA_MOMENTUM + \
                               rate * (1 - Constants.MA_MOMENTUM)
            self.last_connect_time = time.time()
            self.is_connected = True
        elif not Constants.DEBUG: # no killing when debug
            if time.time() - self.last_connect_time > self.disconnect_duration:
                self.is_connected = False
            if not self.is_connected:
                if not self.process.is_alive():
                    self.logger.error("process died, restart it")
                    self.restart_process()
                elif time.time() - self.last_connect_time > self.restart_duration:
                    self.logger.error('process disconnected too long, '
                                      'kill and restart')
                    self.restart_process()

        if time.time() - self.last_update_time > Constants.UPDATE_PERIOD:
            self.logger.info(f"update rate {self.update_rate:4.1f}hz")
            self.last_update_time = time.time()

    def restart_process(self):
        if not self.process.is_alive():
            self.process.terminate()
        self.process = self.new_process_fn()
        self.process.start()
        self.last_connect_time = time.time()