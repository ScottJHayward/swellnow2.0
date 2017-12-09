import numpy as np
import scipy.interpolate
import pylab

def f14_read(filename):
    #given a fort.14, function returns a list, containing variables [n,x,y,z,e]
    f14=open(filename,'r')
    lines=f14.readlines()
    f14.close
    #read number of nodes and elements 
    num=str(lines[1])
    while '  ' in num:
        num = num.replace('  ', ' ')
    a=num.split(" ")
    print a
    numel=int(a[0])
    numno=int(a[1])
    print "Number of nodes: ", numno
    print "Number of elements: ", numel
    ###########  read nodes, x, y, z     ###################
    node=list()
    lon=list()
    lat=list()
    z=list()
    for i in range(0,numno):
        a=str(lines[i+2])
        while '  ' in a:
            a = a.replace('  ', ' ')
        a=a.strip().split(" ")
        node.insert(int(a[0]),i+1)
        lon.append(float(a[1]))
        lat.append(float(a[2]))
        z.append(float(a[3]))
    print 'first node: ', node[0], 'lon= ', lon[0], 'lat= ', lat[0], 'z= ', z[0]
    print 'last node: ', node[numno-1], 'lon= ', lon[numno-1], 'lat= ', lat[numno-1], 'z= ', z[numno-1]
    print ' ' 
    ##########  create lists nodal elements   ##############
    e=list()
    ei=list()
    ej=list()
    ek=list()

    for i in range(0,numel):
        l=i+numno+2
        a=str(lines[l])
        while '  ' in a:
            a = a.replace('  ', ' ')
        a=a.strip().split(" ")
        e.append(int(a[0]))
        ei.append(int(a[2]))
        ej.append(int(a[3]))
        ek.append(int(a[4]))
    print 'First ele: ', e[0], ei[0], ej[0], ek[0]
    print 'Last ele: ', e[numel-1],ei[numel-1], ej[numel-1], ek[numel-1]

    
    ########### read the open boundaries ################
    
    
        
    return node, lon, lat, z, e, ei, ej, ek, 
   

   
def nodeNeighbors(nodes, elements, ei, ej, ek, fname):
#nodes is number of nodes,
#elements is number of elements
#ei, j, k are above
### returns two-dimensional list. 
### first dimension is the node number, second contains all elements that the list is a part of
### will also print a file, that can bre re-read.
    f1=open(fname, 'w+')
    neighbor=[]
    elems=[]
    mystr=str()
    for n in range(0,nodes):
        print 'node ' , n
        elems[:]=[]
        for e in range(0,elements):
            if ei[e]==n+1 or ej[e]==n+1 or ek[e]==n+1:
                elems.append(e)
                mystr=str(e) + ' '
                f1.write(mystr)
        print elems 
        f1.write('\n')
        neighbor.append(elems[:])
    f1.close
    return elems

def read_ele_table(nodes,fname):
    #reads element table generated 
    ele=[]
    file=open(fname,'r')
    lines=file.readlines()
    for n in range(0,nodes):
        a=str(lines[n])
        a=a.strip().split(" ")
        ele.append(a[:])
    return ele

def pts_in(xmin, xmax, ymin, ymax, xp, yp):
    #given a set of boundaries, returns indices of points with a node inside boundaries
    inside=list()
    for i in range(0,len(node)):
        if xmin <= xp[i] <= xmax and ymin <= yp[i] <= ymax:
            inside.append(i)
    return inside    

def ele_in(xmin, xmax, ymin, ymax, xp, yp, ele, e, ei, ej, ek):
    #return all the elements within a bounded rectangle
    pi=pts_in(xmin, xmax, ymin, ymax, xp, yp)
    ei=[]
    for i in range(0,len(pi)):
        for j in range(0,len(ele[pi[i]])):
            tmp=int(ele[pi[i]][j])
            ei.append(tmp)
    se=set(ei)
    ei=list(se)
    return ei
    
def pts_in_around(e_ins,ei,ej,ek,lon,lat,z):
    #return all points assosiated with the elemets in a polygon
    x=list()
    y=list()
    z2=list()
    node=list()
    for i in range(0,len(e_ins)):
        node.append(ei[e_ins[i]-1])
        node.append(ej[e_ins[i]-1])
        node.append(ek[e_ins[i]-1])
    no=set(node)
    node=list(no)
    #print 'node= ' , node
    for i in range(0,len(node)):
        x.append(lon[node[i]-1])
        y.append(lat[node[i]-1])
        z2.append(z[node[i]-1])
    return x, y, z2
    
def points_inside_ele(xp,yp,ele,ei,ej,ek,n_lon,n_lat,n_z):
    #barycentric method, 
    #xp,yp must be lists, cannot be a single point
    x1=n_lon[ei[ele]]
    y1=n_lat[ei[ele]]
    x2=n_lon[ej[ele]]
    y2=n_lat[ej[ele]]
    x3=n_lon[ek[ele]]
    y3=n_lat[ek[ele]]
    z=np.empty(len(xp))
    z[:]=np.nan
    for i in range(0,len(xp)):
        x=xp[i]
        y=yp[i]
        a = ((y2 - y3)*(x - x3) + (x3 - x2)*(y - y3)) / ((y2 - y3)*(x1 - x3) + (x3 - x2)*(y1 - y3))
        b = ((y3 - y1)*(x - x3) + (x1 - x3)*(y - y3)) / ((y2 - y3)*(x1 - x3) + (x3 - x2)*(y1 - y3))   
        c = 1 - a - b
        if 0 <= a <= 1 and 0 <= b <= 1 and 0 <= c <= 1:
            z[i]=1
    return z
    
def interp_ele(xp,yp,elem,ei,ej,ek,n_lon,n_lat,n_z):
    #barycentric method, intepolate all points within an element, return nan if not inside
    #xp,yp must be lists, cannot be a single point
    z=np.empty([len(xp),len(yp)])
    z[:]=np.nan
    for xi in range (0,len(xp)):
        x=xp[xi]
        for yi in range (0,len(yp)):
            y=yp[yi]
            for i in range(0,len(elem)):
                ele=elem[i]
                x1=n_lon[ei[ele]-1]
                y1=n_lat[ei[ele]-1]
                x2=n_lon[ej[ele]-1]
                y2=n_lat[ej[ele]-1]
                x3=n_lon[ek[ele]-1]
                y3=n_lat[ek[ele]-1]
                a = ((y2 - y3)*(x - x3) + (x3 - x2)*(y - y3)) / ((y2 - y3)*(x1 - x3) + (x3 - x2)*(y1 - y3))
                b = ((y3 - y1)*(x - x3) + (x1 - x3)*(y - y3)) / ((y2 - y3)*(x1 - x3) + (x3 - x2)*(y1 - y3))   
                c = 1 - a - b
                if 0 <= a <= 1 and 0 <= b <= 1 and 0 <= c <= 1:
                    xx1=x2-x1
                    xx2=x3-x1
                    yy1=y2-y1
                    yy2=y3-y1
                    xxp=x-x1
                    yyp=y-y1
                    det=(yy2*xx1-xx2*yy1)
                    t=(xxp*yy2-xx2*yyp)/det
                    u=(xx1*yyp-yy1*xxp)/det
                    #linear interpolation between z[ei] z[ej] z[ek]
                    z[xi,yi]=n_z[ei[ele]-1]+t*(n_z[ej[ele]-1]-n_z[ei[ele]-1])+u*(n_z[ek[ele]-1]-n_z[ei[ele]-1])
    return z
    
def interp_ele_fast(xq, yq, ele, e, ei, ej, ek, xp, yp, z):
    #ins = ele_in(min(xq), max(xq), min(yq), max(yq), xp, yp, ele, e, ei, ej, ek)
    x,y,z2 = pts_in_around (ele,ei,ej,ek,xp,yp,z)
    #barycentric method, intepolate all points within an element, return nan if not inside
    #xp,yp must be lists, cannot be a single point
    points=[]
    for i in range(0,len(x)):
        p=[x[i],y[i]]
        points.append(p)
    print 'points: ' , len(points)
    print 'z: ' , len(z2)
    xg,yg=np.meshgrid(xq,yq)
    zz=scipy.interpolate.griddata(points,z2,(xg,yg),method='linear')
    for xi in range(0,len(xq)):
        for yi in range(0,len(yq)):
            if zz[xi,yi] <= 1:
                zz[xi,yi]=np.nan
    return zz    
    

node, lon, lat, z, e, ei, ej, ek = f14_read('NE_geo.14')

lon1=-71.55
lon2=-71.35
lat1=41.45
lat2=41.5

ins = pts_in(lon1,lon2,lat1,lat2, lon, lat)
#print ins
print len(ins)

elems=read_ele_table(len(node),'neighbor_NE.txt')

e_ins=ele_in(lon1,lon2,lat1,lat2, lon, lat, elems, e, ei, ej, ek)
print len(e_ins)

#elems=nodeNeighbors(len(node), len(e), ei, ej, ek, 'neighbor_NE.txt')
print e_ins[-1]

print e[len(e)-1]

print lon[0]
print lon[len(lon)-1]

#zz = interp_ele(np.linspace(-71.7,-71.5,num=100),np.linspace(41.1, 41.3,num=100),e_ins,ei,ej,ek,lon,lat,z)
zz = interp_ele_fast(np.linspace(lon1,lon2,num=256),np.linspace(lat1,lat2,num=256),e_ins,e,ei,ej,ek,lon,lat,z)
print zz
    
import numpy as np
import matplotlib.pyplot as plt

z3=np.flipud(zz)

plt.imshow(z3)
plt.show()
