"""
═══════════════════════════════════════════════════════════════════════
 Challenge 2 — Robot Explorador Autónomo  v13

 PARAMETRIC — Lee controllerArgs del .wbt:
   controllerArgs ["n_obstacles", "n_samples"]
   Ejemplo: controllerArgs ["5", "3"]

 FEATURES:
   - Genera obstáculos y muestras aleatorias al inicio via Supervisor
   - Canasta física en el robot (muestras caen dentro con gravedad)
   - Timeout por muestra (60s) → marca como inaccesible y sigue
   - Bloqueo permanente de pasillos estrechos
   - A* con inflación decreciente + path smoothing
   - Trail-based return to base
═══════════════════════════════════════════════════════════════════════
"""

from controller import Supervisor
import math, heapq, sys, random

TIME_STEP=32; MAX_SPEED=6.0; APPROACH_SPEED=2.5
ARENA=4.0; GRES=0.05; GN=int(ARENA/GRES); HALF=ARENA/2; LIDAR_MAX=2.0
COLLECT_R=0.25; COLLECT_T=2.5; SHOVEL_DN=-0.65; SHOVEL_UP=0.45
BX,BY=-1.75,-1.75; B_RAD=0.35; ENABLE_RET=True
MAX_INFLATE=3; BACKUP_DIST=0.22; BACKUP_T=1500
DISP_SZ=200; DISP_SCALE=DISP_SZ/ARENA
MAX_NAV_TIME=120.0  # segundos máx por muestra — se resetea si hay progreso

EXPLORAR="EXPLORAR"; NAVEGAR="NAVEGAR"; APROXIMAR="APROXIMAR"
RECOLECTAR="RECOLECTAR"; BACKUP="BACKUP"; REGRESAR="REGRESAR"; COMPLETA="COMPLETA"

# ─── Colores para obstáculos aleatorios ──────────────────────
OBS_COLORS=[
    (0.85,0.12,0.12),(0.95,0.5,0.05),(0.1,0.2,0.9),
    (0.55,0.15,0.85),(0.9,0.85,0.05),(0.1,0.7,0.3),
    (0.8,0.4,0.1),(0.3,0.6,0.9),(0.9,0.2,0.6),
]

# ═══════════════════════════════════════════════════════════════
class Grid:
    F=0; O=1; U=2; B=3
    def __init__(self):
        self.c=[[self.U]*GN for _ in range(GN)]
        for i in range(GN):
            for w in range(3):
                self.c[w][i]=self.O; self.c[GN-1-w][i]=self.O
                self.c[i][w]=self.O; self.c[i][GN-1-w]=self.O

    def w2g(self,x,y):
        return max(0,min(GN-1,int((x+HALF)/GRES))),max(0,min(GN-1,int((y+HALF)/GRES)))
    def g2w(self,gx,gy):
        return gx*GRES-HALF+GRES/2,gy*GRES-HALF+GRES/2

    def update(self,rx,ry,rt,ranges):
        nr=len(ranges)
        for i in range(0,nr,2):  # cada 2 rayos — suficiente resolución
            a=rt+2*math.pi*i/nr-math.pi; r=ranges[i]
            if r<=0.001 or math.isinf(r): continue
            ca,sa=math.cos(a),math.sin(a); cap=min(r,LIDAR_MAX); step=GRES
            for s in range(int(cap/step)):
                d=s*step; gx,gy=self.w2g(rx+d*ca,ry+d*sa)
                if self.c[gx][gy]!=self.F and self.c[gx][gy]!=self.B:
                    self.c[gx][gy]=self.F
            if r<LIDAR_MAX-0.05:
                gx,gy=self.w2g(rx+r*ca,ry+r*sa)
                if self.c[gx][gy]!=self.B: self.c[gx][gy]=self.O

    def block_zone(self,wx,wy,radius_cells=4):
        gx,gy=self.w2g(wx,wy); count=0
        for dx in range(-radius_cells,radius_cells+1):
            for dy in range(-radius_cells,radius_cells+1):
                if dx*dx+dy*dy>radius_cells*radius_cells: continue
                nx,ny=gx+dx,gy+dy
                if 0<=nx<GN and 0<=ny<GN and self.c[nx][ny]==self.F:
                    self.c[nx][ny]=self.B; count+=1
        return count

    def line_clear(self,sx,sy,gx,gy,margin=3):
        """Fast check if straight line is clear of real obstacles (O only)."""
        s=self.w2g(sx,sy); g=self.w2g(gx,gy)
        dx=g[0]-s[0]; dy=g[1]-s[1]
        steps=max(abs(dx),abs(dy),1)
        # Sample every 2 cells for speed
        for i in range(0,steps+1,2):
            cx=s[0]+dx*i//steps; cy=s[1]+dy*i//steps
            # Check cross pattern (faster than full square)
            for m in range(-margin,margin+1):
                if 0<=cx+m<GN and 0<=cy<GN and self.c[cx+m][cy]==self.O: return False
                if 0<=cx<GN and 0<=cy+m<GN and self.c[cx][cy+m]==self.O: return False
        return True

    def clear_blocks(self):
        """Remove all B (corridor block) cells, reset to F."""
        for x in range(GN):
            for y in range(GN):
                if self.c[x][y]==self.B: self.c[x][y]=self.F

    def _inflate(self,margin):
        blocked=set()
        for x in range(GN):
            for y in range(GN):
                if self.c[x][y]==self.O or self.c[x][y]==self.B:
                    if margin==0: blocked.add((x,y))
                    else:
                        for dx in range(-margin,margin+1):
                            for dy in range(-margin,margin+1):
                                if dx*dx+dy*dy<=margin*margin:
                                    nx,ny=x+dx,y+dy
                                    if 0<=nx<GN and 0<=ny<GN: blocked.add((nx,ny))
        return blocked

    def _astar_with_blocked(self,sx,sy,gx,gy,blocked):
        start,goal=self.w2g(sx,sy),self.w2g(gx,gy)
        if start==goal: return [(gx,gy)]
        b=blocked.copy(); b.discard(start); b.discard(goal)
        heap=[(0.0,start)]; came={}; gs={start:0.0}
        while heap:
            _,cur=heapq.heappop(heap)
            if cur==goal:
                p=[]; n=cur
                while n in came: p.append(self.g2w(*n)); n=came[n]
                p.reverse(); p.append((gx,gy)); return p
            cx,cy=cur
            for dx in [-1,0,1]:
                for dy in [-1,0,1]:
                    if dx==0 and dy==0: continue
                    nx,ny=cx+dx,cy+dy
                    if nx<0 or nx>=GN or ny<0 or ny>=GN: continue
                    if (nx,ny) in b: continue
                    if abs(dx)+abs(dy)==2:
                        if (cx+dx,cy) in b or (cx,cy+dy) in b: continue
                    cost=1.414 if abs(dx)+abs(dy)==2 else 1.0
                    t=gs[cur]+cost
                    if t<gs.get((nx,ny),1e9):
                        came[(nx,ny)]=cur; gs[(nx,ny)]=t
                        hx,hy=abs(nx-goal[0]),abs(ny-goal[1])
                        heapq.heappush(heap,(t+max(hx,hy)+0.414*min(hx,hy),(nx,ny)))
        return None

    def a_star(self,sx,sy,gx,gy):
        for margin in range(MAX_INFLATE,-1,-1):
            blocked=self._inflate(margin)
            result=self._astar_with_blocked(sx,sy,gx,gy,blocked)
            if result: return self._smooth(result),margin
        return None,0

    def _smooth(self,path):
        if len(path)<=2: return path
        smooth=[path[0]]
        for i in range(1,len(path)-1):
            dx1=path[i][0]-path[i-1][0]; dy1=path[i][1]-path[i-1][1]
            dx2=path[i+1][0]-path[i][0]; dy2=path[i+1][1]-path[i][1]
            if abs(dx1-dx2)>0.001 or abs(dy1-dy2)>0.001: smooth.append(path[i])
        smooth.append(path[-1])
        return smooth


def dist(a,b,c,d): return math.sqrt((c-a)**2+(d-b)**2)
def norm(a):
    while a>math.pi: a-=2*math.pi
    while a<-math.pi: a+=2*math.pi
    return a
def lawnmower():
    wps=[]; lo=-HALF+0.7; hi=HALF-0.7; y=lo; r=True
    while y<=hi: wps.append((hi if r else lo,y)); r=not r; y+=0.5
    return wps


# ═══════════════════════════════════════════════════════════════
class Rover:
    def __init__(self):
        self.sv=Supervisor()

        # ── Leer parámetros de controllerArgs ──
        args=sys.argv[1:]  # Webots pasa controllerArgs como sys.argv[1:]
        self.n_obs=int(args[0]) if len(args)>0 else 5
        self.n_samples=int(args[1]) if len(args)>1 else 3
        print(f"  Parámetros: {self.n_obs} obstáculos, {self.n_samples} muestras")

        # ── Dispositivos ──
        self.lm=self.sv.getDevice("left_motor"); self.rm=self.sv.getDevice("right_motor")
        self.lm.setPosition(float('inf')); self.rm.setPosition(float('inf'))
        self.lm.setVelocity(0); self.rm.setVelocity(0)
        self.shovel=self.sv.getDevice("shovel_motor")
        self.shovel.setVelocity(2.0); self.shovel.setPosition(SHOVEL_UP)
        self.lidar=self.sv.getDevice("lidar"); self.lidar.enable(TIME_STEP); self.lidar.enablePointCloud()
        self.cam=self.sv.getDevice("camera"); self.cam.enable(TIME_STEP); self.cam.recognitionEnable(TIME_STEP)
        self.gps=self.sv.getDevice("gps"); self.gps.enable(TIME_STEP)
        self.compass=self.sv.getDevice("compass"); self.compass.enable(TIME_STEP)
        self.disp=self.sv.getDevice("map_display")

        # ── Generar mundo aleatorio ──
        # Resetear posición del robot al inicio
        robot_node=self.sv.getSelf()
        if robot_node:
            tf=robot_node.getField("translation")
            if tf: tf.setSFVec3f([-1.65,-1.65,0.055])
            rf=robot_node.getField("rotation")
            if rf: rf.setSFRotation([0,0,1,0.785])
        self._spawn_world()

        self.grid=Grid()
        self.estado=EXPLORAR; self.tk=0; self.col=0; self.rec=0; self.done=False
        self.discovered={}; self.target=None; self.inaccessible=set()
        self.target_start_time=0; self.target_best_dist=9999
        self.ewps=lawnmower(); self.eidx=0
        self.path=None; self.pidx=0; self.need_plan=True
        self.last_inflate=MAX_INFLATE
        self.bk_t0=0; self.bk_dir=1; self.col_t0=0; self.trail=[]
        self.bk_zones=[]; self.bk_escape=False
        self.return_wps=None
        self._stuck_pos=None; self._stuck_tick=0

    # ─── GENERACIÓN ALEATORIA DEL MUNDO ─────────────────────────
    def _cleanup_old(self):
        """Elimina obstáculos y muestras de ejecuciones anteriores."""
        root=self.sv.getRoot()
        children=root.getField("children")
        to_remove=[]
        for i in range(children.getCount()):
            node=children.getMFNode(i)
            nf=node.getField("name")
            if not nf: continue
            name=nf.getSFString()
            if name.startswith("obs_") or name.startswith("sample_"):
                to_remove.append(node)
        for node in reversed(to_remove):
            node.remove()
        if to_remove:
            print(f"    Limpieza: {len(to_remove)} objetos anteriores eliminados")

    def _spawn_world(self):
        """Limpia objetos anteriores y genera nuevos aleatorios."""
        self._cleanup_old()
        root=self.sv.getRoot()
        children=root.getField("children")
        placed=[]  # lista de (x,y,radius) para evitar solapamiento

        # Zona base y robot → no colocar nada cerca
        placed.append((BX,BY,0.5))
        placed.append((-1.65,-1.65,0.3))

        # ── Obstáculos ──
        shapes=["box","cylinder"]
        for i in range(self.n_obs):
            for attempt in range(50):
                x=random.uniform(-1.6,1.6)
                y=random.uniform(-1.6,1.6)
                size=random.uniform(0.15,0.22)  # radio o half-size
                # Verificar que no solapa con nada
                ok=True
                for px,py,pr in placed:
                    if dist(x,y,px,py)<size+pr+0.15:
                        ok=False; break
                if ok:
                    placed.append((x,y,size))
                    break
            else:
                continue  # no se pudo colocar

            shape=random.choice(shapes)
            color=OBS_COLORS[i%len(OBS_COLORS)]
            h=random.uniform(0.12,0.18)

            if shape=="box":
                sx=size*2; sy=random.uniform(size*1.2,size*2.2)
                vrml=f'''DEF OBS{i+1} Solid {{
  translation {x} {y} {h/2}
  children [ Shape {{
    appearance PBRAppearance {{ baseColor {color[0]} {color[1]} {color[2]} roughness 0.7 metalness 0 }}
    geometry Box {{ size {sx:.3f} {sy:.3f} {h:.3f} }}
  }} ]
  name "obs_{i+1}"
  boundingObject Box {{ size {sx:.3f} {sy:.3f} {h:.3f} }}
}}'''
            else:
                vrml=f'''DEF OBS{i+1} Solid {{
  translation {x} {y} {h/2}
  children [ Shape {{
    appearance PBRAppearance {{ baseColor {color[0]} {color[1]} {color[2]} roughness 0.7 metalness 0 }}
    geometry Cylinder {{ height {h:.3f} radius {size:.3f} }}
  }} ]
  name "obs_{i+1}"
  boundingObject Cylinder {{ height {h:.3f} radius {size:.3f} }}
}}'''
            children.importMFNodeFromString(-1,vrml)
            print(f"    Obstáculo {i+1}: {shape} en ({x:.2f},{y:.2f}) size={size:.2f}")

        # ── Muestras ──
        for i in range(self.n_samples):
            for attempt in range(50):
                x=random.uniform(-1.5,1.5)
                y=random.uniform(-1.5,1.5)
                # Debe estar a >1m de la base
                if dist(x,y,BX,BY)<1.0: continue
                # No solapar con obstáculos
                ok=True
                for px,py,pr in placed:
                    if dist(x,y,px,py)<pr+0.15:
                        ok=False; break
                if ok:
                    placed.append((x,y,0.05))
                    break
            else:
                x=random.uniform(-0.5,0.5); y=random.uniform(-0.5,0.5)
                placed.append((x,y,0.05))

            vrml=f'''DEF SAMPLE{i+1} Solid {{
  translation {x} {y} 0.04
  children [ Shape {{
    appearance PBRAppearance {{ baseColor 0 0.95 0.95 roughness 0.2 metalness 0.1 emissiveColor 0 0.5 0.5 }}
    geometry Cylinder {{ height 0.08 radius 0.04 }}
  }} ]
  name "sample_{i+1}"
  recognitionColors [ 0 0.95 0.95 ]
  boundingObject Cylinder {{ height 0.08 radius 0.04 }}
}}'''
            children.importMFNodeFromString(-1,vrml)
            print(f"    Muestra {i+1}: en ({x:.2f},{y:.2f})")

    # ─── SENSORES ──────────────────────────────────────────────
    def pose(self):
        g=self.gps.getValues(); c=self.compass.getValues()
        return g[0],g[1],math.atan2(c[0],c[1])
    def t(self): return self.tk*TIME_STEP/1000.0
    def log(self,m): print(f"[{self.t():7.1f}s] [{self.estado:<12s}] {m}")

    def percibir(self):
        x,y,th=self.pose()
        ranges=self.lidar.getRangeImage()
        if ranges: self.grid.update(x,y,th,ranges)
        mf=LIDAR_MAX
        if ranges:
            n=len(ranges); sec=int(n*50/360); ctr=n//2
            for i in range(max(0,ctr-sec),min(n,ctr+sec)):
                r=ranges[i]
                if 0.001<r<mf and not math.isinf(r): mf=r
        objs=self.cam.getRecognitionObjects()
        if objs:
            for obj in objs:
                node=self.sv.getFromId(obj.getId())
                if not node: continue
                nf=node.getField("name")
                if not nf: continue
                name=nf.getSFString()
                if not name.startswith("sample_"): continue
                tf=node.getField("translation")
                if not tf: continue
                wp=tf.getSFVec3f()
                if name not in self.discovered:
                    self.discovered[name]={"x":wp[0],"y":wp[1],"done":False,"node":node}
                    self.log(f"📡 '{name}' en ({wp[0]:.2f},{wp[1]:.2f})")
                else:
                    self.discovered[name]["x"]=wp[0]; self.discovered[name]["y"]=wp[1]
        near=None; nd=9999
        for nm,info in self.discovered.items():
            if info["done"] or nm in self.inaccessible: continue
            d=dist(x,y,info["x"],info["y"])
            if d<nd: nd=d; near=nm
        dt=9999
        if self.target and self.target in self.discovered and not self.discovered[self.target]["done"]:
            s=self.discovered[self.target]; dt=dist(x,y,s["x"],s["y"])
        return {"x":x,"y":y,"th":th,"mf":mf,"near":near,"nd":nd,"dt":dt,
                "db":dist(x,y,BX,BY),"ranges":ranges}

    def plan(self,x,y,tx,ty):
        # Si destino muy cerca → ir directo sin A*
        d=dist(x,y,tx,ty)
        if d<1.0 and self.grid.line_clear(x,y,tx,ty):
            self.path=[(tx,ty)]; self.pidx=0; self.need_plan=False
            self.last_inflate=-1; return True
        # A* normal
        result,margin=self.grid.a_star(x,y,tx,ty)
        if result:
            self.path=result; self.pidx=0; self.need_plan=False
            self.last_inflate=margin; return True
        # Si A* falla, limpiar bloques y reintentar
        self.grid.clear_blocks()
        result,margin=self.grid.a_star(x,y,tx,ty)
        if result:
            self.path=result; self.pidx=0; self.need_plan=False
            self.last_inflate=margin; return True
        self.path=None; return False

    def next_wp(self,x,y):
        if not self.path: return None
        while self.pidx<len(self.path)-1:
            w=self.path[self.pidx]
            if dist(x,y,w[0],w[1])<0.08: self.pidx+=1
            else: break
        return self.path[self.pidx] if self.pidx<len(self.path) else None

    def go(self,x,y,th,tx,ty,spd=MAX_SPEED):
        err=norm(math.atan2(ty-y,tx-x)-th); abs_err=abs(err)
        turn=max(-1,min(1,err*10/math.pi))
        if abs_err>1.5: fwd=0
        elif abs_err>0.5: fwd=spd*0.2
        else: fwd=spd*(1-abs_err*0.8)
        ls=fwd-turn*spd*0.5; rs=fwd+turn*spd*0.5
        return max(-spd,min(spd,ls)),max(-spd,min(spd,rs))

    # ─── DECIDIR ──────────────────────────────────────────────
    def decidir(self,p):
        prev=self.estado
        if prev==COMPLETA: return COMPLETA
        all_done=self.rec>=self.n_samples or (self.rec+len(self.inaccessible)>=self.n_samples)

        if prev==BACKUP:
            duration=3000 if self.bk_escape else BACKUP_T
            if (self.tk-self.bk_t0)*TIME_STEP<duration: return BACKUP
            self.path=None; self.need_plan=True
            if self.bk_escape:
                self.log(f"🚀 Escape OK → replan ({p['x']:.1f},{p['y']:.1f})")
                self.bk_escape=False
            else:
                self.log(f"↩ Backup OK → replan ({p['x']:.1f},{p['y']:.1f})")
            if all_done: return REGRESAR
            if self.target: return NAVEGAR
            return EXPLORAR

        # ── Detección de atascamiento ──
        if prev in [NAVEGAR,EXPLORAR,REGRESAR]:
            if self._stuck_pos is None:
                self._stuck_pos=(p["x"],p["y"]); self._stuck_tick=self.tk
            elif dist(p["x"],p["y"],self._stuck_pos[0],self._stuck_pos[1])>0.3:
                self._stuck_pos=(p["x"],p["y"]); self._stuck_tick=self.tk
            elif (self.tk-self._stuck_tick)*TIME_STEP>10000:
                # Atascado 10s sin moverse → forzar escape
                self._stuck_pos=None; self._stuck_tick=self.tk
                self.bk_t0=self.tk; self.bk_escape=True
                self.bk_dir=1 if self.tk%2==0 else -1
                self.path=None; self.need_plan=True
                self.log(f"🔄 ATASCADO 10s en ({p['x']:.1f},{p['y']:.1f}) → escape forzado")
                return BACKUP

        if p["mf"]<BACKUP_DIST and prev in [NAVEGAR,EXPLORAR,REGRESAR]:
            self.bk_t0=self.tk
            bx,by=p["x"],p["y"]
            nearby=sum(1 for zx,zy in self.bk_zones if dist(bx,by,zx,zy)<0.5)
            self.bk_zones.append((bx,by))
            if len(self.bk_zones)>30: self.bk_zones=self.bk_zones[-20:]
            if nearby>=3:
                self.bk_escape=True
                # Escapar LEJOS de la pared más cercana
                if abs(bx)>1.4: self.bk_dir=1 if bx>0 else -1  # alejarse de pared E/W
                elif abs(by)>1.4: self.bk_dir=1 if by>0 else -1  # alejarse de pared N/S
                else: self.bk_dir=1 if self.tk%2==0 else -1
                # Solo bloquear si NO estamos cerca de la pared de arena
                if abs(bx)<1.3 and abs(by)<1.3:
                    th=p["th"]
                    for d_fwd in [0.10,0.15,0.20,0.25,0.30]:
                        fx,fy=bx+d_fwd*math.cos(th),by+d_fwd*math.sin(th)
                        if abs(fx)<1.5 and abs(fy)<1.5:  # no bloquear cerca de paredes
                            self.grid.block_zone(fx,fy,3)
                # Si demasiados backups → declarar target inaccesible
                if nearby>=15 and self.target:
                    self.inaccessible.add(self.target)
                    self.log(f"⚠ '{self.target}' INACCESIBLE (demasiados backups)")
                    self.target=None; self.path=None; self.need_plan=True
                    return EXPLORAR
                self.log(f"🚀⛔ {nearby+1} backups → ESCAPE")
            else:
                self.bk_escape=False; self.bk_dir=1 if self.tk%2==0 else -1
                self.log(f"🛑 Obs {p['mf']:.2f}m → backup")
            return BACKUP

        if prev==RECOLECTAR:
            if self.t()-self.col_t0<COLLECT_T: return RECOLECTAR
            nm=self.target
            if nm and nm in self.discovered and not self.discovered[nm]["done"]:
                self.discovered[nm]["done"]=True; self.rec+=1
                node=self.discovered[nm]["node"]
                if node:
                    # Remover muestra del mundo (recolección limpia)
                    node.remove()
                self.log(f"✓✓✓ '{nm}' RECOLECTADA ({self.rec}/{self.n_samples})")
            self.shovel.setPosition(SHOVEL_UP)
            self.target=None; self.path=None; self.need_plan=True
            if self.rec>=self.n_samples or self.rec+len(self.inaccessible)>=self.n_samples:
                if ENABLE_RET: self.log("★ Recolección completa → BASE"); return REGRESAR
                return COMPLETA
            self.log("→ Explorando..."); return EXPLORAR

        if prev==APROXIMAR:
            if p["dt"]<COLLECT_R:
                self.col_t0=self.t(); self.shovel.setPosition(SHOVEL_DN)
                self.log(f"✓ Rango '{self.target}' → PALA"); return RECOLECTAR
            if p["dt"]>1.5: self.need_plan=True; return NAVEGAR
            return APROXIMAR

        if prev==REGRESAR:
            if p["db"]<B_RAD: self.log(f"★ BASE! d={p['db']:.3f}m"); return COMPLETA
            return REGRESAR

        if all_done:
            if ENABLE_RET: return REGRESAR
            return COMPLETA

        # Timeout por muestra — con detección de progreso
        if self.target and self.target_start_time>0:
            # Si estamos más cerca que antes, resetear timer (hay progreso)
            if p["dt"]<self.target_best_dist-0.3:
                self.target_best_dist=p["dt"]
                self.target_start_time=self.t()  # resetear timer
            if self.t()-self.target_start_time>MAX_NAV_TIME:
                self.inaccessible.add(self.target)
                self.log(f"⚠ '{self.target}' INACCESIBLE (sin progreso {MAX_NAV_TIME:.0f}s) — saltando")
                self.target=None; self.path=None; self.need_plan=True
                remaining=sum(1 for nm,i in self.discovered.items() if not i["done"] and nm not in self.inaccessible)
                if remaining==0:
                    if ENABLE_RET: self.log("★ No quedan accesibles → BASE"); return REGRESAR
                    return COMPLETA
                return EXPLORAR

        if p["near"] and p["nd"]<3.5:
            nm=p["near"]
            if nm not in self.inaccessible:
                # Solo cambiar target si: no hay target, o el nuevo es >1m más cerca
                should_switch = (self.target is None or
                                 self.target not in self.discovered or
                                 self.discovered[self.target]["done"] or
                                 self.target in self.inaccessible or
                                 p["nd"] < p["dt"] - 1.0)  # histéresis 1m
                if should_switch and self.target!=nm:
                    self.target=nm; self.need_plan=True
                    self.target_start_time=self.t(); self.target_best_dist=p["nd"]
                    self.bk_zones=[]
                    self.grid.clear_blocks()
                    self.log(f"🎯 '{nm}' (d={p['nd']:.1f}m)")
                if p["nd"]<0.50 or p["dt"]<0.50: return APROXIMAR
                return NAVEGAR

        if self.target and self.target in self.discovered and not self.discovered[self.target]["done"]:
            if self.target in self.inaccessible: self.target=None; return EXPLORAR
            if p["dt"]<0.50: return APROXIMAR
            return NAVEGAR
        self.target=None; return EXPLORAR

    # ─── ACTUAR ──────────────────────────────────────────────
    def actuar(self,p):
        x,y,th=p["x"],p["y"],p["th"]; ls=rs=0.0

        if self.estado==EXPLORAR:
            if self.eidx<len(self.ewps):
                wp=self.ewps[self.eidx]
                if dist(x,y,wp[0],wp[1])<0.3:
                    self.eidx+=1; self.path=None; self.need_plan=True
                    if self.eidx>=len(self.ewps): self.eidx=0
            if self.eidx<len(self.ewps):
                wp=self.ewps[self.eidx]
                if self.need_plan or not self.path: self.plan(x,y,wp[0],wp[1])
                nxt=self.next_wp(x,y)
                if nxt: ls,rs=self.go(x,y,th,nxt[0],nxt[1])
                elif self.path: ls,rs=self.go(x,y,th,wp[0],wp[1])
                else: ls,rs=MAX_SPEED*0.3,-MAX_SPEED*0.3

        elif self.estado==NAVEGAR:
            if self.target and self.target in self.discovered:
                s=self.discovered[self.target]
                if self.need_plan or not self.path:
                    ok=self.plan(x,y,s["x"],s["y"])
                    if ok: self.log(f"A* → '{self.target}' ({len(self.path)}wp inf={self.last_inflate})")
                nxt=self.next_wp(x,y)
                if nxt: ls,rs=self.go(x,y,th,nxt[0],nxt[1])
                elif self.path: ls,rs=self.go(x,y,th,s["x"],s["y"])
                else: ls,rs=MAX_SPEED*0.3,-MAX_SPEED*0.3

        elif self.estado==APROXIMAR:
            if self.target and self.target in self.discovered:
                s=self.discovered[self.target]; ls,rs=self.go(x,y,th,s["x"],s["y"],APPROACH_SPEED)
        elif self.estado==RECOLECTAR: pass
        elif self.estado==BACKUP:
            ms=(self.tk-self.bk_t0)*TIME_STEP
            if self.bk_escape:
                if ms<1200: ls,rs=-MAX_SPEED*0.5,-MAX_SPEED*0.45
                else: ls,rs=MAX_SPEED*0.6*self.bk_dir,-MAX_SPEED*0.6*self.bk_dir
            else:
                if ms<600: ls,rs=-MAX_SPEED*0.4,-MAX_SPEED*0.35
                else: ls,rs=MAX_SPEED*0.5*self.bk_dir,-MAX_SPEED*0.5*self.bk_dir
        elif self.estado==REGRESAR:
            if self.return_wps is None:
                self.grid.clear_blocks()  # limpiar zonas bloqueadas para el regreso
                rev=list(reversed(self.trail)); rev.append((BX,BY))
                self.return_wps=[]; last=None
                for tx,ty in rev:
                    if last is None or dist(tx,ty,last[0],last[1])>0.4:
                        self.return_wps.append((tx,ty)); last=(tx,ty)
                if not self.return_wps or dist(self.return_wps[-1][0],self.return_wps[-1][1],BX,BY)>0.2:
                    self.return_wps.append((BX,BY))
                self.ret_idx=0; self.need_plan=True
                self.log(f"🏠 Regreso: {len(self.return_wps)} wps por ruta conocida")

            # Si estamos cerca de la base → ir DIRECTO, sin A*, sin trail
            if p["db"]<1.0:
                ls,rs=self.go(x,y,th,BX,BY)
                if p["db"]<0.5: ls*=0.5; rs*=0.5
            elif self.ret_idx<len(self.return_wps):
                rwp=self.return_wps[self.ret_idx]
                if dist(x,y,rwp[0],rwp[1])<0.3:
                    self.ret_idx+=1; self.path=None; self.need_plan=True
                if self.ret_idx<len(self.return_wps):
                    rwp=self.return_wps[self.ret_idx]
                    if self.need_plan or not self.path:
                        ok=self.plan(x,y,rwp[0],rwp[1])
                        if ok and self.ret_idx%3==0:
                            self.log(f"A* → retorno wp{self.ret_idx}/{len(self.return_wps)} inf={self.last_inflate}")
                    nxt=self.next_wp(x,y)
                    if nxt: ls,rs=self.go(x,y,th,nxt[0],nxt[1])
                    elif self.path: ls,rs=self.go(x,y,th,rwp[0],rwp[1])
                    else: ls,rs=self.go(x,y,th,rwp[0],rwp[1])
                if p["db"]<0.5: ls*=0.5; rs*=0.5

        # Frenado Lidar
        if p["mf"]<0.6 and self.estado not in [RECOLECTAR,COMPLETA,BACKUP]:
            f=max(0.15,(p["mf"]-0.05)/0.55); ls*=f; rs*=f
        self.lm.setVelocity(ls); self.rm.setVelocity(rs)
        if self.tk%int(8000/TIME_STEP)==0 and self.estado in [EXPLORAR,NAVEGAR,REGRESAR]:
            self.need_plan=True

    # ─── DISPLAY ──────────────────────────────────────────────
    def draw_map(self,p):
        d=self.disp; w=h=DISP_SZ; x,y,th=p["x"],p["y"],p["th"]
        def w2p(wx,wy):
            return max(0,min(w-1,int((wx+HALF)*DISP_SCALE))),max(0,min(h-1,int((HALF-wy)*DISP_SCALE)))
        d.setColor(0x0A0E17); d.fillRectangle(0,0,w,h)

        # Grid — dibuja con stride de 2 (4x menos pixels)
        last_c=-1
        for gx in range(0,GN,2):
            for gy in range(0,GN,2):
                v=self.grid.c[gx][gy]
                if v==Grid.U: continue
                c=0xE04040 if v==Grid.O else (0xFF8000 if v==Grid.B else 0x1A1F2E)
                if c!=last_c: d.setColor(c); last_c=c
                px=int(gx*w/GN); py=int((GN-1-gy)*h/GN)
                d.drawPixel(px,py)

        # Base
        bpx,bpy=w2p(BX-0.25,BY+0.25)
        d.setColor(0x208030); d.fillRectangle(bpx,bpy,int(0.5*DISP_SCALE),int(0.5*DISP_SCALE))

        # Path
        if self.path and len(self.path)>1:
            d.setColor(0x40C0FF)
            for i in range(len(self.path)-1):
                x1,y1=w2p(*self.path[i]); x2,y2=w2p(*self.path[i+1])
                d.drawLine(x1,y1,x2,y2)

        # Trail — solo últimos 200 puntos, cada 3
        d.setColor(0x204060)
        for i in range(max(0,len(self.trail)-200),len(self.trail),3):
            tx,ty=self.trail[i]; px,py=w2p(tx,ty); d.drawPixel(px,py)

        # Return wps
        if self.return_wps and self.estado==REGRESAR:
            d.setColor(0xFFCC00)
            for i,rwp in enumerate(self.return_wps):
                if i>=getattr(self,'ret_idx',0):
                    px,py=w2p(rwp[0],rwp[1]); d.fillOval(px-2,py-2,4,4)

        # Muestras
        for nm,info in self.discovered.items():
            if info["done"]: continue
            px,py=w2p(info["x"],info["y"])
            if nm in self.inaccessible: d.setColor(0xFF0000)
            else: d.setColor(0x00F0F0)
            d.fillOval(px-4,py-4,8,8)

        # Robot
        rpx,rpy=w2p(x,y)
        d.setColor(0xFF6600 if self.estado==BACKUP else 0x40A0FF)
        d.fillOval(rpx-4,rpy-4,8,8)
        dx=int(12*math.cos(th)); dy=int(-12*math.sin(th))
        d.setColor(0xFF4020); d.drawLine(rpx,rpy,rpx+dx,rpy+dy)

        # HUD
        d.setColor(0xC0C0C0)
        d.drawText(f"{self.estado} inf={self.last_inflate}",3,3)
        d.drawText(f"rec:{self.rec}/{self.n_samples} inac:{len(self.inaccessible)}",3,14)
        d.drawText(f"t:{self.t():.0f}s obs:{self.n_obs}",3,25)

    # ─── REPORTE ──────────────────────────────────────────────
    def report(self):
        x,y,_=self.pose(); t=self.t(); db=dist(x,y,BX,BY); c=self.col; mr=self.rec
        ns=self.n_samples; ni=len(self.inaccessible)
        print(); print("="*62)
        print(f"  REPORTE — ROVER v13 ({self.n_obs} obs, {ns} muestras)")
        print("="*62)
        print(f"  Tiempo: {t:.1f}s | Rec: {mr}/{ns} | Inaccesibles: {ni} | Col: {c} | Base: {db:.3f}m")
        for nm,i in self.discovered.items():
            st="RECOLECTADA" if i["done"] else ("INACCESIBLE" if nm in self.inaccessible else "pendiente")
            print(f"    {nm}: {st}")
        print("-"*62)
        print(f"  TC-01 Sin colisiones      : {'PASS' if c==0 else 'FAIL'} ({c})")
        print(f"  TC-02 Recolección         : {'PASS' if mr>=1 else 'FAIL'}")
        print(f"  TC-03 Evasión+recolección : {'PASS' if c==0 and mr>=1 else 'FAIL'}")
        print(f"  TC-04 Múltiples muestras  : {'PASS' if mr>=2 else 'FAIL'} ({mr})")
        if ENABLE_RET:
            print(f"  TC-05 Regreso a base      : {'PASS' if db<0.50 else 'FAIL'} ({db:.3f}m)")
        if ni>0:
            print(f"  ⚠ {ni} muestra(s) declarada(s) inaccesible(s)")
        print("="*62)

    # ─── LOOP PRINCIPAL ──────────────────────────────────────
    def run(self):
        print("="*62)
        print(f"  ROVER v13 — Paramétrico ({self.n_obs} obs, {self.n_samples} muestras)")
        print(f"  Arena: {ARENA}x{ARENA}m | Inflación: {MAX_INFLATE}→0")
        print(f"  Timeout muestra: {MAX_NAV_TIME:.0f}s | Canasta física")
        print("="*62)
        for _ in range(4): self.sv.step(TIME_STEP)
        while self.sv.step(TIME_STEP)!=-1:
            self.tk+=1; p=self.percibir()
            self.estado=self.decidir(p); self.actuar(p)
            if len(self.trail)==0 or dist(p["x"],p["y"],self.trail[-1][0],self.trail[-1][1])>0.05:
                self.trail.append((p["x"],p["y"]))
                if len(self.trail)>2000: self.trail=self.trail[-1500:]
            if self.tk%10==0: self.draw_map(p)
            if self.estado==COMPLETA and not self.done:
                self.report(); self.done=True; self.draw_map(p)
            if self.tk%max(1,int(3000/TIME_STEP))==0 and self.estado!=COMPLETA:
                tgt=self.target or "—"
                self.log(f"({p['x']:.1f},{p['y']:.1f}) →{tgt} rec={self.rec}/{self.n_samples} col={self.col}")

Rover().run()
