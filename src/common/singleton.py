def singleton(cls):
    instance = {}
    def wrapper(*args, **kwargs):
        if cls not in instance:
            instance[cls] = cls(*args, **kwargs)
        return instance[cls]
    return wrapper

def main():
    @singleton
    class ClsA(object):
        def __init__(self, val):
            self.val = val

    @singleton
    class ClsB(object):
        def __init__(self, val):
            self.val = val

    a = ClsA(12)
    b = ClsA(13)

if __name__ == '__main__':
    main()
