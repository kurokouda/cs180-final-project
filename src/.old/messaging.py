from heapq import heappush, heappop

import pygame

from .entity import EntityManager
from .singleton import SingletonABCMeta


class Telegram(object):
    NO_EXTRA_INFO = None
    SEND_MESSAGE_IMMEDIATELY = 0
    SMALLEST_DELAY = 0.25

    def __init__(self, dispatch_time=-1, sender_id=-1, receiver_id=-1,
            message=None, extra_info=None):
        self.sender_id = sender_id
        self.receiver_id = receiver_id
        self.message = message
        self.extra_info = extra_info
        self.dispatch_time = dispatch_time

    def __hash__(self):
        '''Returns a unique hash for every telegram.

        Telegrams that differ by dispatch time have the same hash.
        '''
        telegram_hash = 3
        telegram_hash = 89 * telegram_hash + int(self.sender_id)
        telegram_hash = 89 * telegram_hash + int(self.receiver_id)
        telegram_hash = 89 * telegram_hash + int(self.message.id)
        return telegram_hash

    def __eq__(self, other):
        return (isinstance(other, Telegram) and
                (abs(self.dispatch_time - other.dispatch_time)
                        < Telegram.SMALLEST_DELAY and
                    self.sender_id == other.sender_id and
                    self.receiver_id == other.receiver_id and
                    self.message == other.message))

    def __lt__(self, other):
        return self.dispatch_time < other.dispatch_time



class MessageDispatcher(object, metaclass=SingletonABCMeta):
    def __init__(self):
        self._pq = []

    def dispatch_message(self, delay, sender_id, receiver_id,
            message, extra_info):
        receiver = EntityManager().get_entity_from_id(receiver_id)
        telegram = Telegram(0, sender_id, receiver_id, message, extra_info)

        if delay <= 0.0:
            self._discharge(receiver, telegram)
        else:
            # Current time in seconds
            current_time = pygame.time.get_ticks() * 0.001
            telegram.dispatch_time = current_time + delay
            heappush(self._pq, telegram)

    def dispatch_delayed_messagess(self):
        '''Dispatches all messages whose delay times have expired.

        Call this on every tick of the game loop.
        '''
        current_time = pygame.time.get_ticks() * 0.001

        while (self._pq and
                self._pq[0].dispatch_time < current_time and
                self._pq[0].dispatch_time > 0):
            telegram = heappop(self._pq)
            receiver = EntityManager().get_entity_from_id(telegram.receiver_id)
            self._discharge(receiver, telegram)

    def _discharge(self, receiver, message):
        if not receiver.handle_message(message):
            print('Message not handled')
