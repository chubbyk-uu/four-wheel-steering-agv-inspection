#!/usr/bin/env python3
"""Lossless candidate codecs on existing material tiles; no production format change."""
import argparse,ctypes,ctypes.util,json,time
from pathlib import Path
import lz4.frame
import numpy as np


class Zstd:
    def __init__(self):
        self.lib=ctypes.CDLL(ctypes.util.find_library('zstd'))
        size=ctypes.c_size_t;ptr=ctypes.c_void_p
        self.lib.ZSTD_compressBound.argtypes=[size];self.lib.ZSTD_compressBound.restype=size
        self.lib.ZSTD_compress.argtypes=[ptr,size,ptr,size,ctypes.c_int];self.lib.ZSTD_compress.restype=size
        self.lib.ZSTD_decompress.argtypes=[ptr,size,ptr,size];self.lib.ZSTD_decompress.restype=size
        self.lib.ZSTD_isError.argtypes=[size];self.lib.ZSTD_isError.restype=ctypes.c_uint
    def compress(self,data,level):
        out=ctypes.create_string_buffer(self.lib.ZSTD_compressBound(len(data)))
        n=self.lib.ZSTD_compress(out,len(out),data,len(data),level)
        if self.lib.ZSTD_isError(n):raise RuntimeError('zstd compress failed')
        return out.raw[:n]
    def decompress(self,data,count):
        out=ctypes.create_string_buffer(count);n=self.lib.ZSTD_decompress(out,count,data,len(data))
        if self.lib.ZSTD_isError(n) or n!=count:raise RuntimeError('zstd decode failed')
        return out.raw


def main():
    p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,default=Path('assets/road/baked_fullwidth_20m_v1/manifest.json'))
    p.add_argument('--output',type=Path,required=True);p.add_argument('--samples',type=int,default=24);a=p.parse_args()
    scene=json.loads(a.scene.read_text());material=scene['ground_material'];stride=material['core_pixels']+2*material['gutter_pixels']
    rng=np.random.default_rng(20260910);indices=rng.choice(len(material['tiles']),min(a.samples,len(material['tiles'])),replace=False)
    lib=Zstd();stats={k:dict(raw=0,compressed=0,encode=[],decode=[],channels={}) for k in ('lz4','zstd1','zstd3','delta_zstd1')}
    sampled=[]
    for index in indices:
        tile=material['tiles'][int(index)];sampled.append([tile['ix'],tile['iy']]);timings={k:[0.,0.] for k in stats}
        for name,channels in (('color',1),('normal',2)):
            raw=(a.scene.parent/tile[name]['file']).read_bytes();shape=(stride,stride,channels)
            for codec,s in stats.items():
                start=time.perf_counter();data=raw
                if codec=='delta_zstd1':
                    pixels=np.frombuffer(raw,np.uint8).reshape(shape);diff=pixels.copy();diff[:,1:]-=pixels[:,:-1];data=diff.tobytes()
                compressed=lz4.frame.compress(data) if codec=='lz4' else lib.compress(data,3 if codec=='zstd3' else 1)
                timings[codec][0]+=time.perf_counter()-start
                repetitions=[]
                for _ in range(3):
                    start=time.perf_counter();decoded=lz4.frame.decompress(compressed) if codec=='lz4' else lib.decompress(compressed,len(raw))
                    if codec=='delta_zstd1':decoded=np.cumsum(np.frombuffer(decoded,np.uint8).reshape(shape),axis=1,dtype=np.uint8).tobytes()
                    repetitions.append(time.perf_counter()-start)
                    assert decoded==raw,codec+' changed pixels'
                timings[codec][1]+=max(repetitions)
                s['raw']+=len(raw);s['compressed']+=len(compressed)
                channel=s['channels'].setdefault(name,dict(raw_bytes=0,compressed_bytes=0));channel['raw_bytes']+=len(raw);channel['compressed_bytes']+=len(compressed)
        for codec,s in stats.items():s['encode'].append(timings[codec][0]);s['decode'].append(timings[codec][1])
    report=dict(scope='Warm CPU in-memory compression/decompression; complete color+normal tile, allocation/copies included; no disk/GPU/prefetch timing',
                sample_count=len(sampled),sampled_tiles_xy=sampled,seed=20260910,all_decoded_bytes_identical=True,methods={})
    for codec,s in stats.items():
        ratio=s['compressed']/s['raw'];report['methods'][codec]=dict(stored_fraction=ratio,compression_ratio=1/ratio,
            estimated_87_54GB_road_gb=87540536160*ratio/1e9,encode_tile_ms_mean=float(np.mean(s['encode'])*1000),
            decode_tile_ms_median=float(np.median(s['decode'])*1000),decode_tile_ms_p95=float(np.percentile(s['decode'],95)*1000),
            decode_tile_ms_max=float(max(s['decode'])*1000),channels=s['channels'])
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report['methods']))

if __name__=='__main__':main()
