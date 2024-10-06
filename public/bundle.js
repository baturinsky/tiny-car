(() => {
  // src/physlib.ts
  var abs = (a2) => Math.abs(a2);
  var Vec2 = (x, y) => ({ x, y });
  var mag = (v) => dot(v, v) ** 0.5;
  var add = (v, w2) => Vec2(v.x + w2.x, v.y + w2.y);
  var sub = (v, w2) => add(v, mul(w2, -1));
  var mul = (v, n) => Vec2(v.x * n, v.y * n);
  var dot = (v, w2) => v.x * w2.x + v.y * w2.y;
  var cross = (v, w2) => v.x * w2.y - v.y * w2.x;
  var rotate = (v, center2, angle, x = v.x - center2.x, y = v.y - center2.y) => Vec2(x * Math.cos(angle) - y * Math.sin(angle) + center2.x, x * Math.sin(angle) + y * Math.cos(angle) + center2.y);
  var normalize = (v) => mul(v, 1 / (mag(v) || 1));
  var distance = (v, w2) => mag(sub(v, w2));
  var mGravity = Vec2(0, 0);
  var objects = [];
  var collisionInfo = {};
  var collisionInfoR1 = {};
  var collisionInfoR2 = {};
  var setInfo = (collision, D, N, S) => {
    collision.D = D;
    collision.N = N;
    collision.S = S;
    collision.E = add(S, mul(N, D));
  };
  var resizeRect = (s, W, H) => {
    s.W = W;
    s.H = H;
    s.X = [
      // Vertex: 0: TopLeft, 1: TopRight, 2: BottomRight, 3: BottomLeft (rectangles)
      Vec2(s.C.x - W / 2, s.C.y - H / 2),
      Vec2(s.C.x + W / 2, s.C.y - H / 2),
      Vec2(s.C.x + W / 2, s.C.y + H / 2),
      Vec2(s.C.x - W / 2, s.C.y + H / 2)
    ];
    if (s.T) {
      let g = s.G;
      rotateShape(s, s.G);
      s.G = g;
    }
  };
  var RigidShape = (C, mass, F, R2, T, B, W = 0, H = 0) => {
    let shape = {
      T,
      // 0 circle / 1 rectangle
      C,
      // center
      F,
      // friction
      R: R2,
      // restitution (bouncing)
      M: mass ? 1 / mass : 0,
      // inverseMass (0 if immobile)
      V: Vec2(0, 0),
      // velocity (speed)
      A: mass ? mGravity : Vec2(0, 0),
      // acceleration
      G: 0,
      // angle
      v: 0,
      // angle velocity
      a: 0,
      // angle acceleration
      B,
      // (bounds) radius
      I: T ? (Math.hypot(W, H) / 2, mass > 0 ? 1 / (mass * (W ** 2 + H ** 2) / 12) : 0) : mass > 0 ? mass * B ** 2 / 12 : 0,
      // circle
      N: []
      // face normals array (rectangles)
    };
    resizeRect(shape, W, H);
    if (mass != void 0)
      objects.push(shape);
    return shape;
  };
  var moveShape = (shape, v, i2) => {
    shape.C = add(shape.C, v);
    if (shape.T) {
      for (i2 = 4; i2--; ) {
        shape.X[i2] = add(shape.X[i2], v);
      }
    }
  };
  var rotateShape = (shape, angle) => {
    shape.G += angle;
    if (shape.T) {
      for (let i2 = 4; i2--; ) {
        shape.X[i2] = rotate(shape.X[i2], shape.C, angle);
      }
      computeRectNormals(shape);
    }
  };
  var boundTest = (s1, s2) => mag(sub(s2.C, s1.C)) <= s1.B + s2.B;
  var computeRectNormals = (shape, i2) => {
    for (i2 = 4; i2--; ) {
      shape.N[i2] = normalize(sub(shape.X[(i2 + 1) % 4], shape.X[(i2 + 2) % 4]));
    }
  };
  var findAxisLeastPenetration = (rect, otherRect, collisionInfo3) => {
    var n, i2, j, supportPoint, bestDistance = 1e9, bestIndex = -1, hasSupport = true, tmpSupportPoint, tmpSupportPointDist;
    for (i2 = 4; hasSupport && i2--; ) {
      n = rect.N[i2];
      var dir = mul(n, -1), ptOnEdge = rect.X[i2], vToEdge, projection;
      tmpSupportPointDist = -1e9;
      tmpSupportPoint = -1;
      for (j = 4; j--; ) {
        vToEdge = sub(otherRect.X[j], ptOnEdge);
        projection = dot(vToEdge, dir);
        if (projection > 0 && projection > tmpSupportPointDist) {
          tmpSupportPoint = otherRect.X[j];
          tmpSupportPointDist = projection;
        }
      }
      hasSupport = tmpSupportPoint !== -1;
      if (hasSupport && tmpSupportPointDist < bestDistance) {
        bestDistance = tmpSupportPointDist;
        bestIndex = i2;
        supportPoint = tmpSupportPoint;
      }
    }
    if (hasSupport) {
      setInfo(collisionInfo3, bestDistance, rect.N[bestIndex], add(supportPoint, mul(rect.N[bestIndex], bestDistance)));
    }
    return hasSupport;
  };
  var testCollision = (c1, c2) => {
    if (!c1.T && !c2.T) {
      var vFrom1to2 = sub(c2.C, c1.C), rSum = c1.B + c2.B, dist = mag(vFrom1to2);
      if (dist <= Math.sqrt(rSum * rSum)) {
        var normalFrom2to1 = normalize(mul(vFrom1to2, -1)), radiusC2 = mul(normalFrom2to1, c2.B);
        setInfo(collisionInfo, rSum - dist, normalize(vFrom1to2), add(c2.C, radiusC2));
      }
      return 1;
    }
    if (c1.T && c2.T) {
      var status1 = false, status2 = false;
      status1 = findAxisLeastPenetration(c1, c2, collisionInfoR1);
      if (status1) {
        status2 = findAxisLeastPenetration(c2, c1, collisionInfoR2);
        if (status2) {
          if (collisionInfoR1.D < collisionInfoR2.D) {
            setInfo(collisionInfo, collisionInfoR1.D, collisionInfoR1.N, sub(collisionInfoR1.S, mul(collisionInfoR1.N, collisionInfoR1.D)));
          } else {
            setInfo(collisionInfo, collisionInfoR2.D, mul(collisionInfoR2.N, -1), collisionInfoR2.S);
          }
        }
      }
      return status1 && status2;
    }
    if (!c1.T && c2.T) {
      [c1, c2] = [c2, c1];
    }
    if (c1.T && !c2.T) {
      var inside = 1, bestDistance = -1e9, nearestEdge = 0, i2, v, circ2Pos, projection;
      for (i2 = 4; i2--; ) {
        circ2Pos = c2.C;
        v = sub(circ2Pos, c1.X[i2]);
        projection = dot(v, c1.N[i2]);
        if (projection > 0) {
          bestDistance = projection;
          nearestEdge = i2;
          inside = 0;
          break;
        }
        if (projection > bestDistance) {
          bestDistance = projection;
          nearestEdge = i2;
        }
      }
      let dis, normal;
      if (inside) {
        setInfo(collisionInfo, c2.B - bestDistance, c1.N[nearestEdge], sub(circ2Pos, mul(c1.N[nearestEdge], c2.B)));
      } else {
        var v1 = sub(circ2Pos, c1.X[nearestEdge]), v2 = sub(c1.X[(nearestEdge + 1) % 4], c1.X[nearestEdge]), dotp = dot(v1, v2);
        if (dotp < 0) {
          dis = mag(v1);
          if (dis > c2.B) {
            return;
          }
          normal = normalize(v1);
          setInfo(collisionInfo, c2.B - dis, normal, add(circ2Pos, mul(normal, -c2.B)));
        } else {
          v1 = sub(circ2Pos, c1.X[(nearestEdge + 1) % 4]);
          v2 = mul(v2, -1);
          dotp = dot(v1, v2);
          if (dotp < 0) {
            dis = mag(v1);
            if (dis > c2.B) {
              return;
            }
            normal = normalize(v1);
            setInfo(collisionInfo, c2.B - dis, normal, add(circ2Pos, mul(normal, -c2.B)));
          } else {
            if (bestDistance < c2.B) {
              setInfo(collisionInfo, c2.B - bestDistance, c1.N[nearestEdge], sub(circ2Pos, mul(c1.N[nearestEdge], c2.B)));
            } else {
              return;
            }
          }
        }
      }
      return 1;
    }
  };
  var resolveCollision = (s1, s2, collisionInfo3) => {
    if (!s1.M && !s2.M) {
      return;
    }
    var num = collisionInfo3.D / (s1.M + s2.M) * 0.8, correctionAmount = mul(collisionInfo3.N, num), n = collisionInfo3.N;
    moveShape(s1, mul(correctionAmount, -s1.M));
    moveShape(s2, mul(correctionAmount, s2.M));
    var start = mul(collisionInfo3.S, s2.M / (s1.M + s2.M)), end = mul(collisionInfo3.E, s1.M / (s1.M + s2.M)), p = add(start, end), r1 = sub(p, s1.C), r2 = sub(p, s2.C), v1 = add(s1.V, Vec2(-1 * s1.v * r1.y, s1.v * r1.x)), v2 = add(s2.V, Vec2(-1 * s2.v * r2.y, s2.v * r2.x)), relativeVelocity = sub(v2, v1), rVelocityInNormal = dot(relativeVelocity, n);
    if (rVelocityInNormal > 0) {
      return;
    }
    let newRestituion = Math.min(s1.R, s2.R), newFriction = Math.min(s1.F, s2.F), R1crossN = cross(r1, n), R2crossN = cross(r2, n), jN = -(1 + newRestituion) * rVelocityInNormal / (s1.M + s2.M + R1crossN * R1crossN * s1.I + R2crossN * R2crossN * s2.I), impulse = mul(n, jN);
    s1.V = sub(s1.V, mul(impulse, s1.M));
    s2.V = add(s2.V, mul(impulse, s2.M));
    s1.v -= R1crossN * jN * s1.I;
    s2.v += R2crossN * jN * s2.I;
    let tangent = mul(normalize(sub(relativeVelocity, mul(n, dot(relativeVelocity, n)))), -1), R1crossT = cross(r1, tangent), R2crossT = cross(r2, tangent), jT = -(1 + newRestituion) * dot(relativeVelocity, tangent) * newFriction / (s1.M + s2.M + R1crossT * R1crossT * s1.I + R2crossT * R2crossT * s2.I);
    if (jT > jN) {
      jT = jN;
    }
    impulse = mul(tangent, jT);
    s1.V = sub(s1.V, mul(impulse, s1.M));
    s2.V = add(s2.V, mul(impulse, s2.M));
    s1.v -= R1crossT * jT * s1.I;
    s2.v += R2crossT * jT * s2.I;
  };
  function computeCollisionsWith(car2) {
    for (let o of objects) {
      if (o != car2)
        collideAB(o, car2);
    }
  }
  function collideAB(a2, b2) {
    if (boundTest(a2, b2)) {
      if (testCollision(a2, b2, collisionInfo)) {
        if (dot(collisionInfo.N, sub(b2.C, a2.C)) < 0) {
          collisionInfo = {
            D: collisionInfo.D,
            N: mul(collisionInfo.N, -1),
            S: collisionInfo.E,
            E: collisionInfo.S
          };
        }
        resolveCollision(a2, b2, collisionInfo);
      }
    }
  }
  function updatePhysics() {
    for (let obj of objects) {
      obj.V = add(obj.V, mul(obj.A, 1 / 60));
      moveShape(obj, mul(obj.V, 1 / 60));
      obj.v += obj.a * 1 / 60;
      rotateShape(obj, obj.v * 1 / 60);
    }
  }
  var Circle = (center2, radius, mass, friction = 0, restitution = 0) => RigidShape(center2, mass, friction, restitution, 0, radius);
  var Rectangle = (center2, width, height, mass, friction = 0, restitution = 0) => RigidShape(center2, mass, friction, restitution, 1, Math.hypot(width, height) / 2, width, height);
  function deleteObjects() {
    objects = objects.filter((o) => !o.del);
  }

  // src/prog.ts
  var texts = {
    intro: `Agent! We have detected an anomaly dubbed "Tiny Creature" in the area. 
As you know, Tiny Creatures "fold" the space around them.
It can cause some issues with vision and handling when you nearby, but we believe you can overcome them without much problems.
You should find and collect the creature, along with the other Tiny Creatures in the area if you find them.
`,
    caught: `You have caught a Tiny Creature. But it's likely there are others in the area.
Look for them, but do not forget, you do not have much time.
Do not stay for long, because the space, altered by Tiny Creature, can change unpredictably after it is caught.
`,
    win: `<h3>WELL DONE!</h3> Looks like that's all Tiny Creature around here. Job well done! It took you $T$ to do it. 
  For the reference 5 minutes is a good result and 3 minutes is very good.`
  };
  var TINY = 1;
  var BLIND = 2;
  var ICE = 3;
  var HIDDEN = 0;
  var KNOWN = 1;
  var TAKEN = 2;
  var c = a.getContext("2d");
  var i;
  var tic = 0;
  var cruiseControl = false;
  var carSize = { w: 20, h: 40 };
  var stableVel;
  var anomalies = [];
  var collected = 0;
  var startTime;
  function formatTime(sec) {
    if (!startTime)
      return "";
    sec ??= ~~((Date.now() - startTime) / 1e3);
    let minutes = ~~(sec / 60);
    return `${~~(sec / 60)}:${`${1e4 + sec % 60}`.substring(3, 5)}`;
  }
  var tinyRadius = 1e3;
  var wScale = 1;
  write(texts.intro);
  var randomSeed = 1;
  function rng() {
    let x = Math.sin(randomSeed) * 1e4;
    randomSeed = (randomSeed + Math.E) % 1e8;
    return x - Math.floor(x);
  }
  function inRange(center2, r) {
    let circle = Circle(center2, r, null);
    return obstacles.filter((o) => testCollision(circle, o));
  }
  function vistoHSL(visibility) {
    return `hsl(0 0 ${100 - 100 * visibility})`;
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
    c.translate(-car.C.x, -car.C.y);
    c.translate(viewSize * 0.5 * wScale / xscale, viewSize * 0.5 * wScale / xscale);
    for (let i2 in skidMarks) {
      let prev = Vec2(0, 0);
      if (skidMarks[i2].length > 1e3) {
        skidMarks[i2].splice(0, 100);
      }
      for (let sm of skidMarks[i2]) {
        c.lineWidth = 5 * sm.s * (sm.d ? 1 : 0.4);
        c.strokeStyle = sm.d ? fillStyle : trackStyle;
        if (distance(sm.at, prev) < 50 * sm.s) {
          c.beginPath();
          c.moveTo(sm.at.x, sm.at.y);
          c.lineTo(prev.x, prev.y);
          c.stroke();
        }
        prev = sm.at;
      }
    }
    for (let obj of objects) {
      c.save();
      c.translate(obj.C.x, obj.C.y);
      c.rotate(obj.G);
      c.fillStyle = fillStyle;
      if (!obj.T) {
        c.beginPath();
        c.arc(0, 0, obj.B, 0, 7);
        c.closePath();
        c.fill();
      } else {
        c.fillRect(-obj.W / 2, -obj.H / 2, obj.W, obj.H);
        if (obj == car) {
          c.fillStyle = "#fff";
          c.fillRect(-obj.W / 4, -obj.H / 2 * 0.8, obj.W / 2, obj.H * 0.2);
        }
      }
      c.restore();
      obj.V = add(obj.V, mul(obj.A, 1 / 60));
      moveShape(obj, mul(obj.V, 1 / 60));
      obj.v += obj.a * 1 / 60;
      rotateShape(obj, obj.v * 1 / 60);
    }
    c.fillStyle = "#f00a";
    for (let a2 of tinyAnomalies) {
      if (a2.status > 0) {
        let d = sub(a2.at, car.C);
        let scale = Math.max(wScale, 1e-4);
        if (mag(d) > scale * 1e3) {
          d = mul(d, scale * 700 / mag(d));
        }
        let at = add(car.C, d);
        c.font = `${wScale * 60}px Verdana`;
        c.fillText(a2.status == 1 ? "?" : "X", at.x - wScale * 15, at.y);
      }
    }
    for (let a2 of tinyAnomalies) {
      if (a2.status == TAKEN)
        continue;
      c.fillStyle = "#faa";
      c.beginPath();
      c.arc(a2.at.x + rng() * 1e-3 - 5e-4, a2.at.y + rng() * 1e-3 - 5e-4, 2e-4, 0, 7);
      c.closePath();
      c.fill();
    }
    c.restore();
  }
  function capVel(m) {
    const max = 1e3 * wScale;
    if (mag(m.V) > max) {
      m.V = mul(m.V, max / mag(m.V));
    }
    if (abs(m.v) > 10) {
      m.v = 0;
    }
  }
  window.addEventListener("keydown", (e) => {
    if (e.code == "Space") {
      window.onOk();
    }
    pressed[e.code] = 1;
  });
  window.addEventListener("keyup", (e) => {
    delete pressed[e.code];
  });
  window.onOk = () => {
    note.style.display = "none";
  };
  function f2(n) {
    return ~~(n * 100) / 100;
  }
  function fv2(n) {
    return Object.values(n).map(f2).join(" ");
  }
  function clip(a2, x, b2) {
    return x < a2 ? a2 : x > b2 ? b2 : x;
  }
  function closestAnomaly(kind) {
    let dist = 1e6, anomaly;
    for (let a2 of anomalies) {
      let d = distance(a2.at, car.C);
      if ((kind == 0 || a2.kind == kind) && d < dist) {
        dist = d;
        anomaly = a2;
      }
    }
    return [anomaly, dist];
  }
  function write(text) {
    note.style.display = "flex";
    noteText.innerHTML = text;
  }
  function updateStatus() {
    stdiv.innerHTML = `<big>${collected}/${tinyAnomalies.length}</big><br>Tiny Creatures<br/>caught<br/>${formatTime()}`;
  }
  setInterval(
    () => {
      tic++;
      updateStatus();
      let mults = [0, 1, 2, 3, 4].map((kind) => {
        let [anomaly, dist] = closestAnomaly(kind);
        if (kind == TINY && anomaly.status != TAKEN && dist < 1e-3) {
          if (collected == 0)
            startTime = Date.now();
          collected++;
          anomaly.status = TAKEN;
          if (collected == tinyAnomalies.length) {
            write(texts.win.replace("$T$", formatTime()));
            okbut.style.display = "none";
          } else {
            write(texts.caught);
          }
        }
        return clip(1e-6, Math.atan(2 * dist / anomaly?.r) / 1.5, 1);
      });
      wScale = mults[TINY];
      let handling = mults[ICE];
      let brakes = pressed.Space;
      let l = clip(0, mag(car.V) / wScale - 10, 10);
      car.a = l * (pressed.KeyA ? -1 : pressed.KeyD ? 1 : 0);
      car.v *= 0.9;
      let accelerate = handling * 200 * (wScale + 1e-5), decelerate = handling * 100 * (wScale + 1e-5);
      let vel = mag(car.V);
      if (!pressed.KeyW && !pressed.KeyS && stableVel && cruiseControl) {
        car.A = vel < stableVel ? mul(car.N[0], accelerate) : mul(car.N[0], -decelerate);
      } else {
        car.A = pressed.KeyW ? mul(car.N[0], accelerate) : pressed.KeyS ? mul(car.N[0], -decelerate) : Vec2(0, 0);
      }
      let relVel = { fwd: dot(car.N[0], car.V), right: dot(car.N[1], car.V) };
      let convert = 0.5;
      car.A = add(car.A, mul(car.N[0], convert * abs(relVel.right)));
      car.A = add(car.A, mul(car.N[1], relVel.fwd * -convert));
      let drifting = !!(brakes || abs(relVel.right) > 150 * wScale);
      if (tic % 3 == 0 && mag(car.V) > wScale * 10) {
        for (let i2 of [0, 1, 2, 3]) {
          skidMarks[i2] ||= [];
          skidMarks[i2].push(
            {
              at: add(add(car.C, mul(car.N[0], car.H * 0.4 * (i2 % 2 ? -1 : 1))), mul(car.N[1], car.W * 0.2 * (i2 > 1 ? -1 : 1))),
              s: wScale,
              d: drifting
            }
          );
        }
      }
      let friction = {
        fwd: -((abs(relVel.fwd) * 1 + 100 * wScale) * (brakes ? 5 : 1) * 0.1) * Math.sign(relVel.fwd) * handling,
        right: -(abs(relVel.right) * 4 + 10 * wScale) * Math.sign(relVel.right) * handling
      };
      car.A = add(add(car.A, mul(car.N[0], friction.fwd)), mul(car.N[1], friction.right));
      car && computeCollisionsWith(car);
      capVel(car);
      if (dot(car.V, car.N[0]) < 30 || brakes) {
        stableVel = 0;
      } else if (pressed.KeyW || pressed.KeyS) {
        stableVel = mag(car.V);
      }
      b.innerHTML = [
        `sv ${stableVel}`,
        drifting ? "DRIFTING" : "ND",
        `x:${~~car.C.x} y:${~~car.C.y}`,
        `V ${~~mag(car.V)}`,
        `handling ${handling}`,
        ` relVel ${fv2(relVel)}`,
        `friction ${fv2(friction)} `,
        `scale ${wScale}`
      ].map((a2) => `<div>${a2}</div>`).join("");
      updatePhysics();
      render(Math.max(0, mults[BLIND] * 1.3 - 0.3));
    },
    16
  );
  var R = 3e3;
  var w = 20;
  var wr = 5e3;
  Rectangle(Vec2(0, -wr), wr * 2, w, 0, 1, 0.5);
  Rectangle(Vec2(0, wr), wr * 2, w, 0, 1, 0.5);
  Rectangle(Vec2(-wr, 0), w, wr * 2, 0, 1, 0.5);
  Rectangle(Vec2(wr, 0), w, wr * 2, 0, 1, 0.5);
  var obstacles = [];
  var skidMarks = [];
  var center = Vec2(0, 0);
  function addObst(pos, scale) {
    let shape;
    if (rng() > 0.3) {
      let size = [scale * (rng() + 1) * 100, scale * (rng() + 1) * 600];
      shape = Rectangle(pos, ...size, size[0] * size[1] / 100 * 0, 0, 0.5);
      rotateShape(shape, rng() * 6.3);
    } else {
      shape = Circle(pos, scale * (rng() + 1) * 200, 0, 1, 0.5);
    }
    obstacles.push(shape);
  }
  var cols = 20;
  for (i = cols ** 2; i--; ) {
    let pos = Vec2(
      (~~(i / cols) + rng()) * R * 2 / cols - R,
      (~~(i % cols) + rng()) * R * 2 / cols - R
    );
    addObst(pos, rng() * 0.3);
  }
  for (let ti = 0; ti < 5; ti++) {
    let at = Vec2((rng() * 1.4 - 0.7) * R, (rng() * 1.4 - 0.7) * R);
    anomalies.push({
      at,
      kind: TINY,
      r: tinyRadius,
      status: HIDDEN
    });
  }
  var tinyAnomalies = anomalies.filter((a2) => a2.kind == TINY);
  var dc = 0;
  for (let a2 of tinyAnomalies) {
    let ir = inRange(a2.at, 50);
    for (let o of ir) {
      if (o.T)
        o.del = true;
    }
  }
  console.log(dc);
  obstacles = obstacles.filter((o) => !o.del);
  deleteObjects();
  for (let ac of tinyAnomalies) {
    let ar = R * 0.3;
    for (i = 120; i--; ) {
      let scale = 0.1 ** (rng() * 6);
      let a2 = rng() * Math.PI * 2;
      let at = Vec2(ac.at.x + Math.cos(a2) * scale * ar, ac.at.y + Math.sin(a2) * scale * ar);
      if (i < 20) {
        anomalies.push({
          at,
          kind: i < 10 ? ICE : BLIND,
          r: 700 * scale,
          status: HIDDEN
        });
      }
      addObst(at, scale);
    }
  }
  var car = Rectangle(Vec2(0, R - 50), carSize.w, carSize.h, 10, 0.1, 0.5);
  var sta = tinyAnomalies.sort((a2, b2) => distance(a2.at, car.C) - distance(b2.at, car.C));
  sta[0].status = KNOWN;
  var pressed = {};
})();
