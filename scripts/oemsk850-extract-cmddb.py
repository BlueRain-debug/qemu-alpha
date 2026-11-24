#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Rebuild static ARC/BCM groups from the 17047 OEMSK SDM845 AOP ELF.

SDM845 2.1 selects AOP's 0x1001 tables (Thumb b005598). Dynamic VRM
entries are deliberately absent; the original AOP firmware is not run.
"""
import argparse,hashlib,json,struct
from pathlib import Path

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('aop',type=Path)
parser.add_argument('--output-prefix',type=Path,required=True)
args=parser.parse_args()
source=args.aop.resolve(strict=True)
prefix=args.output_prefix.resolve()
raw=source.read_bytes()
expected='0644ad7dbe21fa3c63af0e84114a25c3b27717e0a262375c161fcbbab2938f79'
if hashlib.sha256(raw).hexdigest()!=expected:
    raise ValueError('Unsupported AOP build; table locations are unverified')
phoff=struct.unpack_from('<I',raw,28)[0]
stride,count=struct.unpack_from('<HH',raw,42)
segments=[struct.unpack_from('<8I',raw,phoff+i*stride) for i in range(count)]

def read(address,size):
    if address<0x100000:address+=0xb000000
    for kind,offset,virtual,physical,length,memory,flags,align in segments:
        if kind==1 and virtual<=address and address+size<=virtual+length:
            return raw[offset+address-virtual:offset+address-virtual+size]
    raise ValueError(hex(address))

def u32(address):return struct.unpack('<I',read(address,4))[0]
def u16(address):return struct.unpack('<H',read(address,2))[0]

db=bytearray(0x90);struct.pack_into('<II',db,0,1,0x0c0330db)
groups=[]
for slot,(slave,header,data,entries,length) in enumerate((
    (3,0xe039c,0xe0398,0xe572a,0xe572c),
    (5,0xe0394,0xe0390,0xe5726,0xe5728),
)):
    n,aux=u16(entries),u16(length)
    headers=read(u32(header),n*24);payload=read(u32(data),aux)
    header_offset=len(db)-0x90;db+=headers
    data_offset=len(db)-0x90;db+=payload
    struct.pack_into('<5H',db,8+slot*16,slave,header_offset,data_offset,n,0x1001)
    resources=[]
    for i in range(n):
        name= headers[i*24:i*24+8].rstrip(b'\0').decode('ascii')
        address,length,offset=struct.unpack_from('<IHH',headers,i*24+16)
        if (address>>16)&7!=slave or offset+length>len(payload):
            raise ValueError('Malformed native resource '+name)
        resources.append({'name':name,'address':hex(address),'aux_bytes':length,
                          'aux':payload[offset:offset+length].hex()})
    groups.append({'slave':slave,'version':'0x1001','resources':resources})
prefix.parent.mkdir(parents=True,exist_ok=True)
out=Path(str(prefix)+'.bin');out.write_bytes(db)
Path(str(prefix)+'-provenance.json').write_text(json.dumps({
    'source':str(source),'source_sha256':expected,
    'selection':'AOP b005598 nonzero HW_VERSION major selects tables version 0x1001',
    'missing':'Dynamic VRM group 4 is not reconstructed',
    'output_sha256':hashlib.sha256(db).hexdigest(),'bytes':len(db),'groups':groups},indent=2))
print('Extracted static native ARC/BCM groups:',len(db),'bytes',sum(len(g['resources']) for g in groups),'resources')
