from types import MethodType

class FooClass(object):
    def __init__(self, name):
        self._name = name

    def foo(self):
        print('My name is', self._name)

def new_foo(self):
    print('My name is', self._name, 'and I love boobs')

def __main__():
    f = FooClass('boobies')
    f.foo()
    f.foo = MethodType(new_foo, f)
    f.foo()

if __name__ == '__main__':
    __main__()
