import sys
from uuid import uuid4
from enum import Enum, unique
from itertools import count

from src.entity import BaseGameEntity
from src.states import StateMachine, SingletonStateABC, StateABC
from src.entity import EntityManager
from src.messaging import MessageDispatcher, Telegram

class BColors(object):
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# @unique
class EntityNames(Enum):
    DRAGON = 0
    TIGER = 1

    @property
    def id(self):
        return self.value

    @property
    def name(self):
        n = None
        if self.value == 0:
            n = 'Dooragoon'
        elif self.value == 1:
            n = 'Tooora'
        return n

@unique
class MessageType(Enum):
    TEASE = 0
    ROAR = 1

    @property
    def id(self):
        return self.value


class Dragon(BaseGameEntity):
    MAX_ENERGY = 5
    BOREDOM_THRESHOLD = 3

    @staticmethod
    def log(message):
        print(BColors.OKGREEN + message + BColors.ENDC)

    class State(object):
        class Wander(SingletonStateABC):
            def enter(self, entity):
                Dragon.log(entity.name + ': ' + 'Gonna stroll...')

            def execute(self, entity):
                Dragon.log(entity.name + ': ' + 'strolling... strolling...')
                entity.energy -= 1
                entity.boredom += 1

                if entity.energy <= 0:
                    entity.state_machine.change_state(Dragon.State.GoHunting())
                elif entity.boredom >= Dragon.BOREDOM_THRESHOLD:
                    entity.state_machine.change_state(Dragon.State.Tease())

            def exit(self, entity):
                Dragon.log(entity.name + ': ' + 'Found something to do.')

            def on_message(self, entity, telegram):
                return False


        class GoHunting(SingletonStateABC):
            def enter(self, entity):
                Dragon.log(entity.name + ': ' + 'Will hunt now...')

            def execute(self, entity):
                Dragon.log(entity.name + ': ' + 'Eating Yum...')
                entity.energy += 1

                if entity.energy >= Dragon.MAX_ENERGY:
                    entity.state_machine.change_state(Dragon.State.Wander())

            def exit(self, entity):
                Dragon.log(entity.name + ': ' + 'I\'m full')

            def on_message(self, entity, telegram):
                return False

        class Tease(SingletonStateABC):
            def enter(self, entity):
                Dragon.log(entity.name + ': ' + 'Will now tease...')

                MessageDispatcher().dispatch_message(
                        delay=Telegram.SEND_MESSAGE_IMMEDIATELY,
                        sender_id=entity.id,
                        receiver_id=EntityNames.TIGER.id,
                        message=MessageType.TEASE,
                        extra_info={
                            'message': 'FUCK YOU!!! HAHAHAHA',
                            }
                        )

            def execute(self, entity):
                entity.boredom = 0
                entity.energy -= 1
                entity.state_machine.change_state(Dragon.State.Wander())

            def exit(self, entity):
                pass

            def on_message(self, entity, telegram):
                return False



    def __init__(self, data):
        super().__init__(data.id)
        self.name = data.name
        self.energy = Dragon.MAX_ENERGY
        self.boredom = 0

        self.state_machine = StateMachine(self)
        self.state_machine.current_state = Dragon.State.Wander()

    def update(self):
        self.state_machine.update()

    def handle_message(self, telegram):
        return self.state_machine.handle_message(telegram)




class Tiger(BaseGameEntity):
    MAX_ENERGY = 7
    class State(object):
        class Wander(SingletonStateABC):
            def enter(self, entity):
                print(entity.name + ':', 'Gonna stroll...')

            def execute(self, entity):
                print(entity.name + ':', 'strolling... strolling...')
                entity.energy -= 1

                if entity.energy <= 0:
                    entity.state_machine.change_state(Tiger.State.GoHunting())

            def exit(self, entity):
                print(entity.name + ':', 'Found something to do.')

            def on_message(self, entity, telegram):
                if telegram.message == MessageType.TEASE:
                    print(entity.name + ':', 'YOU FUCKING DRAGON!!!')
                    return True
                return False


        class GoHunting(SingletonStateABC):
            def enter(self, entity):
                print(entity.name + ':', 'Will hunt now...')

            def execute(self, entity):
                print(entity.name + ':', 'Eating Yum...')
                entity.energy += 1

                if entity.energy >= Tiger.MAX_ENERGY:
                    entity.state_machine.change_state(Tiger.State.Wander())

            def exit(self, entity):
                print(entity.name + ':', 'I\'m full')

            def on_message(self, entity, telegram):
                if telegram.message == MessageType.TEASE:
                    print(entity.name + ':', 'YOU FUCKING DRAGON!!!')
                    return True
                return False


    def __init__(self, data):
        super().__init__(data.id)
        self.name = data.name
        self.energy = Tiger.MAX_ENERGY

        self.state_machine = StateMachine(self)
        self.state_machine.current_state = Tiger.State.Wander()

    def update(self):
        self.state_machine.update()

    def handle_message(self, telegram):
        return self.state_machine.handle_message(telegram)


def main():
    dragon = Dragon(EntityNames.DRAGON)
    tiger = Tiger(EntityNames.TIGER)

    EntityManager().register_entity(dragon)
    EntityManager().register_entity(tiger)

    for i in range(30):
        dragon.update()
        tiger.update()


if __name__ == '__main__':
    main()
