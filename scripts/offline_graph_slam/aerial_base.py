"""Georeferenced GSI aerial background; never fit trajectories to photograph pixels."""
import json,math,urllib.request,io,hashlib
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from PIL import Image
from pyproj import Transformer
from common import ROOT
OUT=ROOT/'aerial';Z=18;SIZE=256;EARTH=6378137.;L=math.pi*EARTH
origin=np.array(json.loads((ROOT/'evaluation.json').read_text())['origin_llh'])
lonlat_to_xyz=Transformer.from_crs(4979,4978,always_xy=True);xyz_to_lonlat=Transformer.from_crs(4978,4979,always_xy=True);to_merc=Transformer.from_crs(4326,3857,always_xy=True)
lat,lon=np.radians(origin[:2]);sl,cl,so,co=np.sin(lat),np.cos(lat),np.sin(lon),np.cos(lon)
R=np.array([[-so,co,0],[-sl*co,-sl*so,cl],[cl*co,cl*so,sl]])
O=np.array(lonlat_to_xyz.transform(origin[1],origin[0],origin[2]));ORIGIN_MERC=np.array(to_merc.transform(origin[1],origin[0]));SCALE=np.cos(lat)
def llh(enu):
 a=np.asarray(enu);a=np.c_[a,np.zeros(len(a))] if a.shape[1]==2 else a;xyz=a@R+O;lon,lat,h=xyz_to_lonlat.transform(*xyz.T);return np.c_[lon,lat,h]
def xy(enu):
 coord=llh(enu);x,y=to_merc.transform(coord[:,0],coord[:,1]);return (np.c_[x,y]-ORIGIN_MERC)*SCALE

def tile_index(lon,lat,z):
 n=2**z;return int((lon+180)/360*n),int((1-math.asinh(math.tan(math.radians(lat)))/math.pi)/2*n)
def fetch(url,path):
 if path.exists():return path.read_bytes()
 req=urllib.request.Request(url,headers={'User-Agent':'Tsukuba2026-offline-aerial-evaluation/1.0'})
 data=urllib.request.urlopen(req,timeout=30).read();path.write_bytes(data);return data

def prepare():
 OUT.mkdir(exist_ok=True);(OUT/'tiles').mkdir(exist_ok=True)
 corners=np.array([[-150,-115],[545,245]]);a=llh(corners);x0,y1=tile_index(a[0,0],a[0,1],Z);x1,y0=tile_index(a[1,0],a[1,1],Z)
 tasks=[(x,y) for y in range(y0,y1+1) for x in range(x0,x1+1)]
 def one(t):
  x,y=t;url=f'https://cyberjapandata.gsi.go.jp/xyz/seamlessphoto/{Z}/{x}/{y}.jpg';path=OUT/'tiles'/f'{Z}_{x}_{y}.jpg';data=fetch(url,path);im=Image.open(io.BytesIO(data)).convert('RGB');assert im.size==(256,256);return x,y,im,{'url':url,'sha256':hashlib.sha256(data).hexdigest()}
 canvas=Image.new('RGB',((x1-x0+1)*SIZE,(y1-y0+1)*SIZE));records=[]
 with ThreadPoolExecutor(max_workers=4) as pool:
  for x,y,im,record in pool.map(one,tasks):canvas.paste(im,((x-x0)*SIZE,(y-y0)*SIZE));records.append(record)
 canvas.save(OUT/'aerial_background.png')
 extent=[(-L+x0*2*L/2**Z-ORIGIN_MERC[0])*SCALE,(-L+(x1+1)*2*L/2**Z-ORIGIN_MERC[0])*SCALE,(L-(y1+1)*2*L/2**Z-ORIGIN_MERC[1])*SCALE,(L-y0*2*L/2**Z-ORIGIN_MERC[1])*SCALE]
 # GSI photography metadata uses native zoom 11.
 xm,ym=tile_index(origin[1],origin[0],11);url=f'https://maps.gsi.go.jp/xyz/seamlessphoto_spec/11/{xm}/{ym}.geojson';spec=json.loads(fetch(url,OUT/'photo_metadata.geojson'))
 manifest={'tile_layer':'seamlessphoto','source':'国土地理院 地理院タイル','source_index':'https://maps.gsi.go.jp/development/ichiran.html','zoom':Z,'ground_pixel_m':2*L/2**Z/256*SCALE,'extent':extent,'origin_llh':origin.tolist(),'display_crs':'EPSG:3857 scaled by cos(origin latitude), relative to origin; approximately local meters','tiles':records,'metadata_url':url,'metadata_properties':[f.get('properties') for f in spec['features']]}
 (OUT/'aerial_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2));print(json.dumps({k:v for k,v in manifest.items() if k!='tiles'},ensure_ascii=False,indent=2))
if __name__=='__main__':prepare()
