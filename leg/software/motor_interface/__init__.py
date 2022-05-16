import moteus
import asyncio
from numpy import pi
class ConnectionError(Exception):
    pass

def get_motor_angles(state):
    return [2*pi*m.values[moteus.Register.POSITION] for m in state]

def get_motor_velocities(state):
    return [2*pi*m.values[moteus.Register.VELOCITY] for m in state]

def get_motor_torques(state):
    return [m.values[moteus.Register.TORQUE] for m in state]

def get_motor_temperature(state):
    return [m.values[moteus.Register.TEMPERATURE] for m in state]

class MoteusInterface():
    m0 = None
    m1 = None
    COMM_TIMEOUT = 1.
    def __init__(self,*ids):
        self.motors = [moteus.Controller(id) for id in ids]

    async def connect(self):
        for m in self.motors:
            try:
                response = await asyncio.wait_for(m.query(), MoteusInterface.COMM_TIMEOUT) 
            except :
                raise(ConnectionError("failed to connect to motor at id %d" %(m.id)))

    async def query_state(self):
        return [await m.query() for m in self.motors]        

    async def set_current(self, *i):
        for j in range(len(i)):
            await self.motors[j].set_current(d_A = 0,q_A = i[j], query=False)

    async def stop(self):
        for m in self.motors:
            await m.make_stop()
       
async def main():
    moteus_interface = MoteusInterface(42)
    await moteus_interface.connect()

if __name__ == "__main__":
    asyncio.run(main())
