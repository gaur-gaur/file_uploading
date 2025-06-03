import asyncio
from aiocoap import *

async def main():
    context = await Context.create_client_context()

    payload = b'</3/0>,</6/0>'  # Objects: Device and Location

    request = Message(
        code=POST,
        uri='coap://192.168.1.17/rd?ep=device123&lt=300&lwm2m=1.0',
        payload=payload
    )
    request.opt.content_format = 40  # application/link-format

    try:
        response = await context.request(request).response
        print("Registered with Leshan server!")
        print("Code:", response.code)
        print("Payload:", response.payload.decode() if response.payload else "")

        for opt in response.opt.option_list():
            if opt.number == 8:  # Location-Path
                print("Location-Path:", opt.value)
    except Exception as e:
        print("Failed to register:", e)

asyncio.run(main())
