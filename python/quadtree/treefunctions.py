import math
import numpy as np
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
import matplotlib as mpl


def geo2world(lon, lat):
    tsize = 256
    siny = math.sin(lat * math.pi / 180)
    siny = min(max(siny, -0.9999), 0.9999);
    num = lon * 0.017453292519943295
    x = 6378137.0 * num
    xw = (x + 20037507.0671618) * tsize / (20037507.0671618 * 2)
    yw = tsize * (0.5 - math.log((1 + siny) / (1 - siny)) / (4 * math.pi)) 
    return xw, yw

def geo2pix(lon,lat,zoom):
	#100 percent accurate in comparison to Google, up to Zoom level 17
    tsize = 256
    siny = math.sin(lat * math.pi / 180)
    siny = min(max(siny, -0.9999), 0.9999);
    num = lon * 0.017453292519943295
    x = 6378137.0 * num
    px = math.floor( (x + 20037507.0671618) * tsize / (20037507.0671618 * 2) * math.pow(2, zoom) )
    py = math.floor( tsize * (0.5 - math.log((1 + siny) / (1 - siny)) / (4 * math.pi)) * math.pow(2, zoom) )
    return px, py

def deg2num(lon_deg, lat_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
    return (xtile, ytile)
  
def num2deg(xtile, ytile, zoom):
    n = 2.0 ** zoom
    lon_deg = xtile / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
    lat_deg = math.degrees(lat_rad)
    return (lat_deg, lon_deg)
  
def tileGrid(xtile,ytile,zoom):
    #makes a meshgrid of the specified tile, in tile coordinates. 
    tsize = 256
    x = range(tsize * xtile, tsize * xtile + tsize)
    y = range(tsize * ytile, tsize * ytile + tsize)
    Tx, Ty = np.meshgrid( x, y )
    return Tx, Ty

def setfig():
    fig = plt.figure(frameon=False)
    ax = plt.Axes(fig, [0., 0., 1., 1.])
    ax.set_axis_off()
    ax.hold(False)
    fig.add_axes(ax)
    fig.set_size_inches((2.56,2.56)) 
    extent = mpl.transforms.Bbox(((0, 0), (2.56,2.56))) 
    return fig, ax, extent
    
def tile2png(data,fname,fig,ax,extent):
    g2=data
    ax.imshow(g2, cmap='jet' , vmax=50., vmin=0.)
    fig.savefig(fname, bbox_inches=extent, transparent=True)
    print ("Printed file: " + fname )
	
def makeLeaf(lon,lat,val,xtile,ytile,zoom,fig,ax,extent):
    #creates image for a single leaf
    ### - Lon and Lat correspond to the 'Val' variable, which will be plotted using pcolor. 
    ### - xtile and ytile are the tile locations, use 'deg2num' to find this if needed
    ### - zoom is the quad tree level
    #First, create a meshgrid from the data
    xp,yp = tileGrid(xtile,ytile,zoom)
    #create a tuple of the points
    L=len(lon)
    Px = list()
    Py = list()
    for p in range(L):
        px, py = geo2pix(lon[p],lat[p],zoom)
        Px.append(px)
        Py.append(py)
    px=np.array(Px)
    py=np.array(Py)
    points=tuple([px,py])
    #interpolate the data
    #if zoom < 7:
    grid= griddata(points,val,(xp,yp),method='linear')
    #else: 
    #grid= griddata(points,val,(xp,yp),method='nearest')
    #fix this later##################################################
    grid[grid <= 1]=np.nan
    #grid[grid >= 50]=50
    #plot figure
    #fname=("./images/" + str(zoom) + "/" + str(zoom) + "z" + str(xtile) + "x" + str(ytile) + "y.png" )
    fname=("./images/" + str(zoom) + "z" + str(xtile) + "x" + str(ytile) + "y.png" )
    tile2png(grid,fname,fig,ax,extent)

def whichLeaf(lon,lat,zoom):
    #find all Leaves that contain data, given a zoom
    L=len(lon)
    coord = tuple()
    point=list()
    for p in range(L):
        tx, ty = deg2num(lon[p],lat[p],zoom)
        coord=(tx,ty)
        point.append(coord)
    tiles=set(point)
    xtile=list()
    ytile=list()
    l2= len(tiles)
    i = iter(tiles)
    for n in range(len(tiles)):
        c = next(i)
        xtile.append(c[0])
        ytile.append(c[1])
    return xtile, ytile 


x=[]
y=[]
x=[-71, -70.75, -70.5, -70.25]
y=[41, 41.25, 41.5, 42]

x2, y2 = whichLeaf(x,y,8)

print x
print x2
print y2
print list
##test point in chicago
#lat= 41.85
#lon= -87.64999999999998
#### test the functions   
#wx, wy = geo2world(lon,lat)
#z=15
#print "world x:", wx
#print "world y:", wy
#px, py = geo2pix(lon, lat, z)
#print "x pixel:", px
#print "y pixel:", py
#tx,ty = deg2num(lon, lat, z)
#print "x tile:", tx
#print "y tile:", ty 

