(()=>{var M=a.getContext("2d"),u,E=0,W=e=>Math.abs(e),V=(e,t)=>({x:e,y:t}),$=e=>D(e,e)**.5,v=(e,t)=>V(e.x+t.x,e.y+t.y),h=(e,t)=>v(e,n(t,-1)),n=(e,t)=>V(e.x*t,e.y*t),D=(e,t)=>e.x*t.x+e.y*t.y,Q=(e,t)=>e.x*t.y-e.y*t.x,le=(e,t,r,f=e.x-t.x,i=e.y-t.y)=>V(f*Math.cos(r)-i*Math.sin(r)+t.x,f*Math.sin(r)+i*Math.cos(r)+t.y),q=e=>n(e,1/($(e)||1)),ae=(e,t)=>$(h(e,t)),ie=V(0,0),l=[],C={},H={},P={},F=(e,t,r,f)=>{e.D=t,e.N=r,e.S=f,e.E=v(f,n(r,t))},te=(e,t,r,f,i,m,x=0,y=0)=>{let N={T:i,C:e,F:r,R:f,M:t?1/t:0,V:V(0,0),A:t?ie:V(0,0),G:0,v:0,a:0,B:m,W:x,H:y,I:i?(Math.hypot(x,y)/2,t>0?1/(t*(x**2+y**2)/12):0):t>0?t*m**2/12:0,N:[],X:[V(e.x-x/2,e.y-y/2),V(e.x+x/2,e.y-y/2),V(e.x+x/2,e.y+y/2),V(e.x-x/2,e.y+y/2)]};return i&&re(N),l.push(N),N},Z=(e,t,r)=>{if(e.C=v(e.C,t),e.T)for(r=4;r--;)e.X[r]=v(e.X[r],t)},oe=(e,t,r)=>{if(e.G+=t,e.T){for(r=4;r--;)e.X[r]=le(e.X[r],e.C,t);re(e)}},fe=(e,t)=>$(h(t.C,e.C))<=e.B+t.B,re=(e,t)=>{for(t=4;t--;)e.N[t]=q(h(e.X[(t+1)%4],e.X[(t+2)%4]))},j=(e,t,r)=>{var f,i,m,x,y=1e9,N=-1,p=!0,X,T;for(i=4;p&&i--;){f=e.N[i];var S=n(f,-1),R=e.X[i],w,d;for(T=-1e9,X=-1,m=4;m--;)w=h(t.X[m],R),d=D(w,S),d>0&&d>T&&(X=t.X[m],T=d);p=X!==-1,p&&T<y&&(y=T,N=i,x=X)}return p&&F(r,y,e.N[N],v(x,n(e.N[N],y))),p},ue=(e,t,r)=>{if(!e.T&&!t.T){var f=h(t.C,e.C),i=e.B+t.B,m=$(f);if(m<=Math.sqrt(i*i)){var x=q(n(f,-1)),y=n(x,t.B);F(C,i-m,q(f),v(t.C,y))}return 1}if(e.T&&t.T){var N=!1,p=!1;return N=j(e,t,H),N&&(p=j(t,e,P),p&&(H.D<P.D?F(C,H.D,H.N,h(H.S,n(H.N,H.D))):F(C,P.D,n(P.N,-1),P.S))),N&&p}if(!e.T&&t.T&&([e,t]=[t,e]),e.T&&!t.T){var X=1,T=-1e9,S=0,R,w,d,z;for(R=4;R--;){if(d=t.C,w=h(d,e.X[R]),z=D(w,e.N[R]),z>0){T=z,S=R,X=0;break}z>T&&(T=z,S=R)}let I,A;if(X)F(C,t.B-T,e.N[S],h(d,n(e.N[S],t.B)));else{var k=h(d,e.X[S]),L=h(e.X[(S+1)%4],e.X[S]),G=D(k,L);if(G<0){if(I=$(k),I>t.B)return;A=q(k),F(C,t.B-I,A,v(d,n(A,-t.B)))}else if(k=h(d,e.X[(S+1)%4]),L=n(L,-1),G=D(k,L),G<0){if(I=$(k),I>t.B)return;A=q(k),F(C,t.B-I,A,v(d,n(A,-t.B)))}else if(T<t.B)F(C,t.B-T,e.N[S],h(d,n(e.N[S],t.B)));else return}return 1}},Ve=(e,t,r)=>{if(!e.M&&!t.M)return;var f=r.D/(e.M+t.M)*.8,i=n(r.N,f),m=r.N;Z(e,n(i,-e.M)),Z(t,n(i,t.M));var x=n(r.S,t.M/(e.M+t.M)),y=n(r.E,e.M/(e.M+t.M)),N=v(x,y),p=h(N,e.C),X=h(N,t.C),T=v(e.V,V(-1*e.v*p.y,e.v*p.x)),S=v(t.V,V(-1*t.v*X.y,t.v*X.x)),R=h(S,T),w=D(R,m);if(w>0)return;let d=Math.min(e.R,t.R),z=Math.min(e.F,t.F),k=Q(p,m),L=Q(X,m),G=-(1+d)*w/(e.M+t.M+k*k*e.I+L*L*t.I),I=n(m,G);e.V=h(e.V,n(I,e.M)),t.V=v(t.V,n(I,t.M)),e.v-=k*G*e.I,t.v+=L*G*t.I;let A=n(q(h(R,n(m,D(R,m)))),-1),U=Q(p,A),Y=Q(X,A),O=-(1+d)*D(R,A)*z/(e.M+t.M+U*U*e.I+Y*Y*t.I);O>G&&(O=G),I=n(A,O),e.V=h(e.V,n(I,e.M)),t.V=v(t.V,n(I,t.M)),e.v-=U*O*e.I,t.v+=Y*O*t.I};function me(){let e,t,r;for(r=9;r--;)for(e=l.length;e--;)for(t=l.length;t-- >e;)fe(l[e],l[t])&&ue(l[e],l[t],C)&&(D(C.N,h(l[t].C,l[e].C))<0&&(C={D:C.D,N:n(C.N,-1),S:C.E,E:C.S}),Ve(l[e],l[t],C))}function Me(){for(M.save(),M.translate(500-o.C.x,500-o.C.y),u=l.length;u--;)M.save(),M.translate(l[u].C.x,l[u].C.y),M.rotate(l[u].G),l[u].T?(M.strokeRect(-l[u].W/2,-l[u].H/2,l[u].W,l[u].H),M.strokeRect(-l[u].W/4,-l[u].H/2*.8,l[u].W/2,l[u].H*.2)):(M.beginPath(),M.arc(0,0,l[u].B,0,7),M.lineTo(0,0),M.closePath(),M.stroke()),M.restore(),l[u].V=v(l[u].V,n(l[u].A,1/60)),Z(l[u],n(l[u].V,1/60)),l[u].v+=l[u].a*1/60,oe(l[u],l[u].v*1/60);M.lineWidth=5,M.strokeStyle="#0008";let e=V(0,0);for(let t of ne){if(ae(t,e)<30){let r=q(h(t,e));r=n(V(r.y,-r.x),5);for(let f of[-1,1])M.beginPath(),M.moveTo(t.x+r.x*f,t.y+r.y*f),M.lineTo(e.x+r.x*f,e.y+r.y*f),M.stroke()}e=t}M.lineWidth=1,M.restore()}function s(e){$(e.V)>1e3&&(e.V=V(0,0)),W(e.v)>10&&(e.v=0)}setInterval(()=>{E++;let e=g[" "],t=be(0,($(o.V)-10)*.1,10);o.a=t*(g.a?-1:g.d?1:0)*2,o.v=(W(o.v)*.9-(e?0:.1))*Math.sign(o.v);let r=500,f=250;o.A=g.w?n(o.N[0],r):g.s?n(o.N[0],-f):V(0,0);let i={x:D(o.N[0],o.V),y:D(o.N[1],o.V)},m=.5;o.A=v(o.A,n(o.N[0],m*W(i.y))),o.A=v(o.A,n(o.N[1],i.x*-m));let x=e||W(i.y)>200;x&&E%3==0&&ne.push(o.C);let y={x:-(W(i.x)*(e?4:1)*.25+20)*Math.sign(i.x),y:-(W(i.y)*(x?1:e?4:10)+10)*Math.sign(i.y)};o.A=v(v(o.A,n(o.N[0],y.x)),n(o.N[1],y.y)),b.innerHTML=`${x?"DRIFTING":""} V ${~~$(o.V)} v ${c(o.v)} relVel ${ee(i)} friction ${ee(y)} `,a.width^=0,me();for(let N of _)N.v*=.95,N.V=n(N.V,.95),s(N);s(o),Me()},16);var ye=(e,t,r,f,i)=>te(e,r,f,i,0,t),K=(e,t,r,f,i,m)=>te(e,f,i,m,1,Math.hypot(t,r)/2,t,r),B=2e3,J=20;K(V(B,J),B*2,J,0,1,.5);K(V(B,B*2),B*2,J,0,1,.5);K(V(J,B),J,B*2,0,1,.5);K(V(B*2,B),J,B*2,0,1,.5);var _=[],ne=[];for(u=100;u--;){let e=[Math.random()*50+50,Math.random()*100+100],t=K(V(Math.random()*B*2,Math.random()*B*2),...e,e[0]*e[1]/100*0,0,.5);t.G=Math.random()*6.3,_.push(t),t=ye(V(Math.random()*B*2,Math.random()*B*2),Math.random()*30+30,0,1,.5),_.push(t)}var o=K(V(500,500),20,40,10,.1,.5),g={};window.addEventListener("keydown",e=>g[e.key]=1,!1);window.addEventListener("keyup",e=>delete g[e.key],!1);function c(e){return~~(e*100)/100}function ee(e){return`${c(e.x)} ${c(e.y)}`}function be(e,t,r){return t<e?e:t>r?r:t}})();
