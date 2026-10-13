import sys, numpy as np
for p in sys.argv[1:]:
    m=np.load(p); fps=50.0; bp=m["body_pos_w"]; bq=m["body_quat_w"]; n=len(bp); t=np.arange(n)/fps
    z=bp[:,:,2].min(1)-0.035   # altura del pie mas bajo sobre su valor de apoyo
    qr=bq[:,0]; w=np.degrees(2*np.arccos(np.clip(np.abs((qr[1:]*qr[:-1]).sum(1)),0,1)))*fps
    # picos aislados (salto) frente a giro sostenido
    hi=w>400; iso=sum(1 for i in np.where(hi)[0] if w[max(0,i-3):i+4].mean()<0.5*w[i])
    k=0; runs=[]
    while k<len(w):
        if hi[k]:
            j=k
            while j<len(w) and hi[j]: j+=1
            runs.append((round(t[k],2),j-k)); k=j
        else: k+=1
    # guinada acumulada
    yaw=np.unwrap(np.arctan2(2*(qr[:,0]*qr[:,3]+qr[:,1]*qr[:,2]),1-2*(qr[:,2]**2+qr[:,3]**2)))
    print(p.split('/')[-1], "| pie mas bajo sobre apoyo: media %.3f p95 %.3f max %.3f m | frac>0.10 %.2f >0.20 %.2f >0.25 %.2f"%(z.mean(),np.percentile(z,95),z.max(),(z>0.10).mean(),(z>0.20).mean(),(z>0.25).mean()))
    print("   velang raiz >400deg/s: %d cuadros en %d tramos, picos aislados %d, tramos(t,cuadros)=%s"%(hi.sum(),len(runs),iso,runs[:14]))
    print("   guinada total recorrida %.0f deg, neta %.0f deg"%(np.degrees(np.abs(np.diff(yaw)).sum()),np.degrees(yaw[-1]-yaw[0])))
    print("   altura pie mas bajo por tramos de 5 s:", [round(float(z[(t>=a)&(t<a+5)].mean()),2) for a in range(0,int(t[-1]),5)])
