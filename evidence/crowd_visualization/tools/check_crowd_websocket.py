import asyncio,json,time,struct
from pathlib import Path
import websockets
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
async def main():
 channels={};received={};schema={};latest={}
 async with websockets.connect('ws://127.0.0.1:8799',subprotocols=['foxglove.sdk.v1'],max_size=8*1024*1024) as ws:
  end=time.monotonic()+12
  while time.monotonic()<end:
   try: raw=await asyncio.wait_for(ws.recv(),1)
   except asyncio.TimeoutError:continue
   if isinstance(raw,str):
    data=json.loads(raw)
    if data.get('op')=='advertise':
     for c in data['channels']:
      if c['topic'].startswith('/multirobot/viz/'):
       i=c['id'];channels[i]=c['topic'];schema[i]=c['schemaName']
       await ws.send(json.dumps({'op':'subscribe','subscriptions':[{'id':i,'channelId':i}]}))
   elif raw and raw[0]==1:
    i=struct.unpack_from('<I',raw,1)[0]
    if i in channels:
     msg=deserialize_message(raw[13:],get_message(schema[i]));name=channels[i]
     received[name]=received.get(name,0)+1
     if hasattr(msg,'people'):latest[name]={'count':len(msg.people),'ids':[p.id for p in msg.people]}
     elif hasattr(msg,'data'):latest[name]=msg.data
 report={'channels':channels,'schema':schema,'received':received,'latest':latest,
  'passed':len(received)==6 and all(v.get('count')==14 for v in latest.values() if isinstance(v,dict))}
 Path('evidence/crowd_visualization/websocket.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
asyncio.run(main())
