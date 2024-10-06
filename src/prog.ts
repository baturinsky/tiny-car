import { distance, resizeRect, add, mul, rotateShape, Vec2, Shape, mag, abs, dot, computeCollisions, RigidShape, updatePhysics, moveShape, objects, computeCollisionsWith, Circle, Rectangle, testCollision, deleteObjects, collisionInfo, sub } from "./physlib";

const texts = {
  intro:
    `Agent! We have detected an anomaly dubbed "Tiny Creature" in the area (marked <span style="color:#F00">?</span>).
As you know, Tiny Creatures "fold" the space around them.
It can cause some issues with vision and handling when you nearby, but we believe you can overcome that without much problems.
You should find and collect the creature, along with the other Tiny Creatures in the area, if you find them.
`,
  caught: `You have caught a Tiny Creature. But it's likely there are others in the area.
Look for them, but do not forget, you do not have much time.
Do not stay for long, because the space, altered by Tiny Creature, can change unpredictably after it is caught.
`,
  win: `<h3>WELL DONE!</h3> Looks like that's all Tiny Creature around here. Job well done! It took you $T$ to do it. 
  For the reference, 5 minutes is a good result and 3 minutes is very good.
  Press SPACE to close this window.`
}



const TINY = 1, BLIND = 2, ICE = 3, HIDDEN = 0, KNOWN = 1, TAKEN = 2;

export type Anomaly = {
  at: Vec2,
  kind: number,
  r: number
  status: number
  removed?: number
}

declare var a: HTMLCanvasElement, b: HTMLDivElement, note: HTMLDivElement, noteText: HTMLDivElement, stdiv: HTMLDivElement, okbut: HTMLDivElement;

let c = a.getContext('2d') as CanvasRenderingContext2D;
let i, tic = 0;
const cruiseControl = false, carSize = { w: 20, h: 40 };
let stableVel: number;
let anomalies: Anomaly[] = [];
let collected = 0;
let startTime: number;

function formatTime(sec?: number) {
  if (!startTime)
    return ""
  sec ??= ~~((Date.now() - startTime) / 1000);
  let minutes = ~~(sec / 60);
  return `${~~(sec / 60)}:${`${(10000 + (sec % 60))}`.substring(3, 5)}`
}

const tinyRadius = 1000;

/**Main View Scale */
let wScale = 1;

write(texts.intro)


let randomSeed = 1;
function rng() {
  let x = Math.sin(randomSeed) * 10000;
  randomSeed = (randomSeed + Math.E) % 1e8;
  return x - Math.floor(x);
}

function inRange(center: Vec2, r: number) {
  let circle = Circle(center, r, null);
  return obstacles.filter(o => testCollision(circle, o))
}

function vistoHSL(visibility) {
  return `hsl(0 0 ${100 - 100 * visibility})`
}

function render(visibility) {
  a.width ^= 0;
  let fillStyle = vistoHSL(visibility);
  let trackStyle = vistoHSL(visibility * 0.5);

  const viewSize = 960;

  resizeRect(car, wScale * carSize.w, wScale * carSize.h);

  c.save();

  let xscale = 0.6;
  c.scale(xscale / wScale, xscale / wScale);
  c.translate(- car.C.x, - car.C.y);
  //c.rotate(-car.G);
  //c.translate(viewSize * (0.5 - 0.1 * Math.sin(car.G)) * wScale, viewSize * (0.5 + 0.1 * Math.cos(car.G)) * wScale);
  c.translate(viewSize * (0.5) * wScale / xscale, viewSize * (0.5) * wScale / xscale);

  for (let i in skidMarks) {
    let prev = Vec2(0, 0);
    if (skidMarks[i].length > 1000) {
      skidMarks[i].splice(0, 100);
    }
    for (let sm of skidMarks[i]) {
      c.lineWidth = 5 * sm.s * (sm.d ? 1 : 0.4);
      c.strokeStyle = sm.d ? fillStyle : trackStyle;
      if (distance(sm.at, prev) < 50 * sm.s) {
        c.beginPath()
        c.moveTo(sm.at.x, sm.at.y);
        c.lineTo(prev.x, prev.y);
        c.stroke();
      }

      prev = sm.at;
    }
  }

  //c.lineWidth = wScale ** 0.75 * 2;

  // Draw / Update scene
  for (let obj of objects) {

    // Draw
    // ----

    c.save();
    c.translate(obj.C.x, obj.C.y);
    c.rotate(obj.G);
    c.fillStyle = fillStyle

    // Circle
    if (!obj.T) {
      c.beginPath();
      c.arc(0, 0, obj.B, 0, 7);
      //c.lineTo(0, 0);
      c.closePath();
      c.fill();
    }

    // Rectangle
    else {//if(obj.T == 1){
      c.fillRect(-obj.W / 2, -obj.H / 2, obj.W, obj.H);
      if (obj == car) {
        c.fillStyle = "#fff"
        c.fillRect(-obj.W / 4, - obj.H / 2 * 0.8, obj.W / 2, obj.H * 0.2);
      }
    }

    c.restore();

    // Update position/rotation
    obj.V = add(obj.V, mul(obj.A, 1 / 60));
    moveShape(obj, mul(obj.V, 1 / 60));
    obj.v += obj.a * 1 / 60;
    rotateShape(obj, obj.v * 1 / 60);
  }

  c.fillStyle = "#f00a"

  for (let a of tinyAnomalies) {
    if (a.status > 0) {
      let d = sub(a.at, car.C)
      let scale = Math.max(wScale, 0.0001)
      if (mag(d) > scale * 1000) {
        d = mul(d, scale * 700 / mag(d))
      }
      let at = add(car.C, d);
      c.font = `${wScale * 60}px Verdana`
      c.globalAlpha = a.status == 1?Math.sin(Date.now()/100)*0.5+0.5:1;
      c.fillText(a.status == 1 ? "?" : "X", at.x - wScale * 15, at.y)
    }
  }
  c.globalAlpha = 1;

  for (let a of tinyAnomalies) {
    if (a.status == TAKEN)
      continue
    c.fillStyle = "#faa"
    c.beginPath();
    c.arc(a.at.x + rng() * 0.001 - 0.0005, a.at.y + rng() * 0.001 - 0.0005, 0.0002, 0, 7);
    c.closePath();
    c.fill();
  }

  c.restore()
}


function capVel(m: Shape) {
  const max = 1000 * wScale;
  if (mag(m.V) > max) {
    m.V = mul(m.V, max / mag(m.V));
  }
  if (abs(m.v) > 10) {
    m.v = 0;
  }
}

window.addEventListener('keydown', e => {
  if (e.code == "Space") {
    window.onOk();
  }
  pressed[e.code] = 1
});

window.addEventListener('keyup', e => {
  delete pressed[e.code]
});

window.onOk = () => {
  note.style.display = "none";
}

function f2(n: number) {
  return ~~(n * 100) / 100;
}

function fv2(n: { [id: string]: number }) {
  return Object.values(n).map(f2).join(" ");
}

function clip(a, x, b) {
  return x < a ? a : x > b ? b : x;
}

function closestAnomaly(kind: number) {
  let dist = 1e6, anomaly!: Anomaly;
  for (let a of anomalies) {
    let d = distance(a.at, car.C);
    if ((kind == 0 || a.kind == kind) && d < dist) {
      dist = d;
      anomaly = a;
    }
  }
  return [anomaly, dist] as [Anomaly, number];
}

function write(text: string) {
  note.style.display = "flex";
  noteText.innerHTML = text;
}

function updateStatus() {
  stdiv.innerHTML = `Tiny Creatures<br/>caught</br><big>${collected}/${tinyAnomalies.length}</big><br/>${startTime?"Time<br/>":""}<big>${formatTime()}</big>`
}

// Main Loop
setInterval(
  () => {
    tic++;
    updateStatus();

    let mults = [0, 1, 2, 3, 4].map(kind => {
      let [anomaly, dist] = closestAnomaly(kind);
      if (kind == TINY && anomaly.status != TAKEN && dist < 0.001) {
        if (collected == 0)
          startTime = Date.now();
        collected++;
        anomaly.status = TAKEN;
        anomaly.removed = Date.now()
        if (collected == tinyAnomalies.length) {
          write(texts.win.replace("$T$", formatTime()));
          okbut.style.display = "none";
        } else {
          write(texts.caught);
        }
      }
      return clip(0.000001, Math.atan(2 * dist / anomaly?.r) / 1.5, 1);
    })

    wScale = mults[TINY];


    let handling = mults[ICE];

    let brakes = pressed.Space

    let l = clip(0, mag(car.V) / wScale - 10, 10);

    let key = {
      f:pressed.KeyW || pressed.ArrowUp,
      b:pressed.KeyS || pressed.ArrowDown,
      l:pressed.KeyA || pressed.ArrowLeft,
      r:pressed.KeyD || pressed.ArrowRight,
    };

    car.a = l * (key.l ? -1 : key.r ? 1 : 0);
    //car.v = (abs(car.v) * 0.9 - (brakes ? 0 : 0.1)) * Math.sign(car.v);
    //car.v = (abs(car.v) * 0.9 - (brakes ? 0 : 0.1)) * Math.sign(car.v) / Math.max(1,wScale);
    car.v *= 0.9;

    let accelerate = handling * 200 * (wScale + 0.00001), decelerate = handling * 100 * (wScale + 0.00001);
    let vel = mag(car.V)

    if (!key.f && !key.b && stableVel && cruiseControl) {
      car.A = vel < stableVel ? mul(car.N[0], accelerate) : mul(car.N[0], -decelerate);
    } else {
      car.A = key.f ? mul(car.N[0], accelerate) : key.b ? mul(car.N[0], -decelerate) : Vec2(0, 0);
    }

    /**Speed along the axes */
    let relVel = { fwd: dot(car.N[0], car.V), right: dot(car.N[1], car.V) };

    let convert = 0.5;
    car.A = add(car.A, mul(car.N[0], convert * abs(relVel.right)))
    car.A = add(car.A, mul(car.N[1], relVel.fwd * (- convert)))

    let drifting = !!(brakes || abs(relVel.right) > 150 * wScale);

    if (tic % 3 == 0 && mag(car.V) > wScale * 10) {
      for (let i of [0, 1, 2, 3]) {
        skidMarks[i] ||= [];
        skidMarks[i].push(
          {
            at: add(add(car.C, mul(car.N[0], car.H * 0.4 * (i % 2 ? -1 : 1))), mul(car.N[1], car.W * 0.2 * (i > 1 ? -1 : 1))),
            s: wScale,
            d: drifting
          }
        )
      }
    }

    /**Friction along the axes */
    let friction = {
      fwd: -((abs(relVel.fwd) * 1 + 100 * wScale) * (brakes ? 5 : 1) * 0.1) * Math.sign(relVel.fwd) * handling,
      right: -(abs(relVel.right) * 4 + 10 * wScale) * Math.sign(relVel.right) * handling
    };

    car.A = add(add(car.A, mul(car.N[0], friction.fwd)), mul(car.N[1], friction.right));


    // Compute collisions    
    car && computeCollisionsWith(car);

    /*for (let m of obstacles) {
      m.v *= 0.95;
      m.V = mul(m.V, 0.95);
      capVel(m)
    }*/

    capVel(car)

    if (dot(car.V, car.N[0]) < 30 || brakes) {
      stableVel = 0
    } else if (key.f || key.b) {
      stableVel = mag(car.V);
    }

    /*
    b.innerHTML = [
      `sv ${stableVel}`,
      drifting ? "DRIFTING" : "ND",
      `x:${~~car.C.x} y:${~~car.C.y}`,
      `V ${~~mag(car.V)}`,
      `handling ${handling}`,
      ` relVel ${fv2(relVel)}`,
      `friction ${fv2(friction)} `,
      `scale ${wScale}`
    ].map(a => `<div>${a}</div>`).join('')*/

    updatePhysics();
    for (let o of obstacles) {
      let removed = o.an?.removed;
      if (removed && o.T) {
        let mult = Math.min(10, 1.0001 ** ((Date.now() - removed) / 1000)) * 0.01;
        rotateShape(o, (rng() - 0.5) * mult);
      }
    }
    render(Math.max(0, mults[BLIND] * 1.3 - 0.3));
  },
  16
);


let R = 3000, w = 20;
let wr = 5000;
Rectangle(Vec2(0, -wr), wr * 2, w, 0, 1, .5);
Rectangle(Vec2(0, wr), wr * 2, w, 0, 1, .5);
Rectangle(Vec2(-wr, 0), w, wr * 2, 0, 1, .5);
Rectangle(Vec2(wr, 0), w, wr * 2, 0, 1, .5);

let obstacles: Shape[] = []
let skidMarks: {
  at: Vec2,
  /**scale */
  s: number,
  /**drifting */
  d: boolean
}[][] = []
let center = Vec2(0, 0);

function addObst(pos: Vec2, scale: number, an?: Anomaly) {
  let shape: Shape;
  if (rng() > 0.3) {
    let size = [scale * (rng() + 1) * 100, scale * (rng() + 1) * 600] as [number, number];
    shape = Rectangle(pos, ...size, size[0] * size[1] / 100 * 0, 0, .5)
    rotateShape(shape, rng() * 6.3)
  } else {
    shape = Circle(pos, scale * (rng() + 1) * 200, 0, 1, .5)
  }
  shape.an = an;
  obstacles.push(shape);
}

let cols = 20;
for (i = cols ** 2; i--;) {
  let pos = Vec2(
    (~~(i / cols) + rng()) * R * 2 / cols - R,
    (~~(i % cols) + rng()) * R * 2 / cols - R
  );
  addObst(pos, rng() * 0.3)
}

for (let ti = 0; ti < 5; ti++) {
  let at = Vec2((rng() * 1.4 - 0.7) * R, (rng() * 1.4 - 0.7) * R);
  anomalies.push({
    at,
    kind: TINY,
    r: tinyRadius,
    status: HIDDEN
  });

  /*at = Vec2((rng() * 1.4 - 0.7) * R, (rng() * 1.4 - 0.7) * R);
  anomalies.push({
    at,
    kind: BLIND,
    r: tinyRadius
  });*/
}

let tinyAnomalies = anomalies.filter(a => a.kind == TINY)

let dc = 0
for (let a of tinyAnomalies) {
  let ir = inRange(a.at, 50);
  for (let o of ir) {
    if (o.T)
      o.del = true
  }
}

console.log(dc);

obstacles = obstacles.filter(o => !o.del)
deleteObjects();

for (let ac of tinyAnomalies) {
  let ar = R * 0.3;
  for (i = 120; i--;) {
    let scale = 0.1 ** (rng() * 6);

    let a = rng() * Math.PI * 2;
    let at = Vec2(ac.at.x + Math.cos(a) * scale * ar, ac.at.y + Math.sin(a) * scale * ar);

    if (i < 20) {
      anomalies.push({
        at,
        kind: i < 10 ? ICE : BLIND,
        r: 700 * scale,
        status: HIDDEN
      });
    } else {
      addObst(at, scale, ac)
    }
  }
}


let car = Rectangle(Vec2(0, R - 50), carSize.w, carSize.h, 10, .1, .5);

let sta = tinyAnomalies.sort((a, b) => distance(a.at, car.C) - distance(b.at, car.C))
sta[0].status = KNOWN;

let pressed: { [id: string]: 1 | undefined } = {};
