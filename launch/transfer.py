import asyncio
import websockets
from bitarray import bitarray

async def receive_and_forward(websocket_from,websocket_to):
    async for message in websocket_from:
        # Convert message to bitarray
        # bit_arr = bitarray()
        # bit_arr.frombytes(message)
        
        # Process bitarray (optional)
        # For example: bit_arr = bit_arr[::-1]  # Reverse bits
        
        # Convert bitarray back to bytes and send
        await websocket_to.send(message)

async def handler():
    uri_from = "ws://localhost:8080"
    #uri_to = "ws://101.32.194.5:8000/ws/push/test"

    uri_to = "ws://101.32.194.5:8000/ws/push/test"
    
    async with websockets.connect(uri_from) as websocket_from, websockets.connect(uri_to) as websocket_to:
        print("连接成功")
        await receive_and_forward(websocket_from, websocket_to)

asyncio.get_event_loop().run_until_complete(handler())




#python3 /home/sunrise/transfer.py