import asyncio
from bleak import BleakScanner


async def connect():
    devices = await BleakScanner.discover()
    for d in devices:  
        print(d)      
        if d.name == 'VESC BLE UART':
            return d

async def main():
    result = await connect()
    print(result)

if __name__ == "__main__":
    asyncio.run(main())