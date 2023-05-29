import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.resolve()))

import cc.AddressBook_pb2 as ab

if __name__ == '__main__':
    person = ab.Person()
    person.name = 'Jack'
    person.id=1245565
    print(f'name: {person.name}')
    print(f'serialize to str: {person.SerializeToString()}')
