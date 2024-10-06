// MINI 2D PHYSICS
// ===============

export type Vec2 = { x: number, y: number };
export let abs = a => Math.abs(a)

export type Collision = {
    /** Depth */
    D: number,
    /** Normal */
    N: Vec2,
    /** Start */
    S: Vec2,
    /** End */
    E: Vec2
};

export type Shape = {
    /** 0 circle / 1 rectangle */
    T: 0 | 1;
    /** Center */
    C: Vec2;
    /** friction */
    F: number;
    /** restitution (bouncing) */
    R: number;
    /** mass ? 1 / mass : 0, // inverseMass (0 if immobile) */
    M: number;
    /** velocity (speed) */
    V: Vec2;
    /** mass ? mGravity : Vec2(0, 0), // acceleration */
    A: Vec2;
    /** angle */
    G: number;
    /** angle velocity */
    v: number;
    /** angle acceleration */
    a: number;
    /** (bounds) radius */
    B: number;
    /** width */
    W: number;
    /** height */
    H: number;
    /** inertia
    ? (Math.hypot(W, H) / 2, mass > 0 ? 1 / (mass * (W ** 2 + H ** 2) / 12) : 0) // rectangle
    : (mass > 0 ? (mass * B ** 2) / 12 : 0), // circle */
    I: number;
    /** face normals array (rectangles) */
    N: Vec2[];
    /** Vertex: 0: TopLeft, 1: TopRight, 2: BottomRight, 3: BottomLeft (rectangles) */
    X: Vec2[];
    del: boolean
}

// 2D vector tools
export let Vec2 = (x, y) => ({ x, y }) as Vec2;
export let mag = (v: Vec2) => dot(v, v) ** .5;
export let add = (v: Vec2, w: Vec2) => Vec2(v.x + w.x, v.y + w.y);
export let sub = (v: Vec2, w: Vec2) => add(v, mul(w, -1));
export let mul = (v: Vec2, n: number) => Vec2(v.x * n, v.y * n);
export let dot = (v: Vec2, w: Vec2) => v.x * w.x + v.y * w.y;
export let cross = (v: Vec2, w: Vec2) => v.x * w.y - v.y * w.x;
export let rotate = (v: Vec2, center: Vec2, angle: number, x = v.x - center.x, y = v.y - center.y) =>
    Vec2(x * Math.cos(angle) - y * Math.sin(angle) + center.x, x * Math.sin(angle) + y * Math.cos(angle) + center.y);
export let normalize = (v: Vec2) => mul(v, 1 / (mag(v) || 1));
export let distance = (v: Vec2, w: Vec2) => mag(sub(v, w));

// Gravity
export let mGravity = Vec2(0, 0);

// All shapes
export let objects: Shape[] = [];

// Collision info 
export let collisionInfo = {} as Collision; // final collision between two shapes
export let collisionInfoR1 = {} as Collision; // temp collision:   ct 1 vs rect 2
export let collisionInfoR2 = {} as Collision; // temp collision: rect 2 vs rect 1

// Collision info setter
let setInfo = (collision, D, N, S) => {
    collision.D = D; // depth
    collision.N = N; // normal
    collision.S = S; // start
    collision.E = add(S, mul(N, D)); // end
};


export let resizeRect = (s: Shape, W: number, H: number) => {
    s.W = W;
    s.H = H;
    s.X = [ // Vertex: 0: TopLeft, 1: TopRight, 2: BottomRight, 3: BottomLeft (rectangles)
        Vec2(s.C.x - W / 2, s.C.y - H / 2),
        Vec2(s.C.x + W / 2, s.C.y - H / 2),
        Vec2(s.C.x + W / 2, s.C.y + H / 2),
        Vec2(s.C.x - W / 2, s.C.y + H / 2)
    ]
    // Prepare rectangle
    if (s.T /* == 1 */) {
        let g = s.G;
        rotateShape(s, s.G)
        s.G = g;
    }

}

/**
 * New shape
 * @param C center
 * @param mass 
 * @param F friction 
 * @param R restitution (bouncing)
 * @param T 0 circle / 1 rectangle
 * @param B (bounds) radius
 * @param W width
 * @param H height
 * @returns 
 */
export let RigidShape = (C: Vec2, mass: number|undefined, F: number, R: number, T: 0 | 1, B: number, W: number = 0, H: number = 0) => {
    let shape = {
        T, // 0 circle / 1 rectangle
        C, // center
        F, // friction
        R, // restitution (bouncing)
        M: mass ? 1 / mass : 0, // inverseMass (0 if immobile)
        V: Vec2(0, 0), // velocity (speed)
        A: mass ? mGravity : Vec2(0, 0), // acceleration
        G: 0, // angle
        v: 0, // angle velocity
        a: 0, // angle acceleration
        B, // (bounds) radius
        I:  T // inertia
            ? (Math.hypot(W, H) / 2, mass > 0 ? 1 / (mass * (W ** 2 + H ** 2) / 12) : 0) // rectangle
            : (mass > 0 ? (mass * B ** 2) / 12 : 0), // circle
        N: [] // face normals array (rectangles)
    } as any as Shape;

    resizeRect(shape, W, H)

    if(mass != undefined)
        objects.push(shape);
    return shape as Shape;
};

// Move a shape along a vector
export let moveShape = (shape, v, i?) => {

    // Center
    shape.C = add(shape.C, v);

    // Rectangle (move vertex)
    if (shape.T) {
        for (i = 4; i--;) {
            shape.X[i] = add(shape.X[i], v);
        }
    }
}

// Rotate a shape around its center
export let rotateShape = (shape: Shape, angle: number) => {

    // Update angle
    shape.G += angle;

    // Rectangle (rotate vertex)
    if (shape.T) {
        for (let i = 4; i--;) {
            shape.X[i] = rotate(shape.X[i], shape.C, angle);
        }
        computeRectNormals(shape);
    }
}


// Test if two shapes have intersecting bounding circles
export let boundTest = (s1, s2) => mag(sub(s2.C, s1.C)) <= s1.B + s2.B;

// Compute face normals (for rectangles)
export let computeRectNormals = (shape, i?) => {

    // N: normal of each face toward outside of rectangle
    // 0: Top, 1: Right, 2: Bottom, 3: Left
    for (i = 4; i--;) {
        shape.N[i] = normalize(sub(shape.X[(i + 1) % 4], shape.X[(i + 2) % 4]));
    }
}

// Find the axis of least penetration between two rects
export let findAxisLeastPenetration = (rect, otherRect, collisionInfo) => {
    var
        n,
        i,
        j,
        supportPoint,
        bestDistance = 1e9,
        bestIndex = -1,
        hasSupport = true,
        tmpSupportPoint,
        tmpSupportPointDist;

    for (i = 4; hasSupport && i--;) {

        // Retrieve a face normal from A
        n = rect.N[i];

        // use -n as direction and the vertex on edge i as point on edge
        var
            dir = mul(n, -1),
            ptOnEdge = rect.X[i],

            // find the support on B
            vToEdge,
            projection;
        tmpSupportPointDist = -1e9;
        tmpSupportPoint = -1;

        // check each vector of other object
        for (j = 4; j--;) {
            vToEdge = sub(otherRect.X[j], ptOnEdge);
            projection = dot(vToEdge, dir);

            // find the longest distance with certain edge
            // dir is -n direction, so the distance should be positive     
            if (projection > 0 && projection > tmpSupportPointDist) {
                tmpSupportPoint = otherRect.X[j];
                tmpSupportPointDist = projection;
            }
        }
        hasSupport = (tmpSupportPoint !== -1);

        // get the shortest support point depth
        if (hasSupport && tmpSupportPointDist < bestDistance) {
            bestDistance = tmpSupportPointDist;
            bestIndex = i;
            supportPoint = tmpSupportPoint;
        }
    }

    if (hasSupport) {

        // all four directions have support point
        setInfo(collisionInfo, bestDistance, rect.N[bestIndex], add(supportPoint, mul(rect.N[bestIndex], bestDistance)));
    }

    return hasSupport;
};

// Test collision between two shapes
export let testCollision = (c1:Shape, c2:Shape) => {

    // Circle vs circle
    if (!c1.T && !c2.T) {
        var
            vFrom1to2 = sub(c2.C, c1.C),
            rSum = c1.B + c2.B,
            dist = mag(vFrom1to2);

        if (dist <= Math.sqrt(rSum * rSum)) {

            //if(dist){

            // overlapping but not same position
            var
                normalFrom2to1 = normalize(mul(vFrom1to2, -1)),
                radiusC2 = mul(normalFrom2to1, c2.B);
            setInfo(collisionInfo, rSum - dist, normalize(vFrom1to2), add(c2.C, radiusC2));
            //}

            /*
            // same position
            else {
              
              if(c1.B > c2.B){
                setInfo(collisionInfo, rSum, Vec2(0, -1), add(c1.C, Vec2(0, c1.B)));
              }
              
              else {
                setInfo(collisionInfo, rSum, Vec2(0, -1), add(c2.C, Vec2(0, c2.B)));
              }
            }
            */
        }

        return 1;
    }

    // Rect vs Rect
    if (c1.T /*== 1*/ && c2.T /*== 1*/) {
        var
            status1 = false,
            status2 = false;

        // find Axis of Separation for both rectangles
        status1 = findAxisLeastPenetration(c1, c2, collisionInfoR1);
        if (status1) {
            status2 = findAxisLeastPenetration(c2, c1, collisionInfoR2);
            if (status2) {

                // if both of rectangles are overlapping, choose the shorter normal as the normal     
                if (collisionInfoR1.D < collisionInfoR2.D) {
                    setInfo(collisionInfo, collisionInfoR1.D, collisionInfoR1.N, sub(collisionInfoR1.S, mul(collisionInfoR1.N, collisionInfoR1.D)));
                }

                else {
                    setInfo(collisionInfo, collisionInfoR2.D, mul(collisionInfoR2.N, -1), collisionInfoR2.S);
                }
            }
        }
        return status1 && status2;
    }

    // Rectangle vs Circle
    // (c1 is the rectangle and c2 is the circle, invert the two if needed)
    if (!c1.T && c2.T /*== 1*/) {
        [c1, c2] = [c2, c1];
    }

    if (c1.T /*== 1*/ && !c2.T) {
        var
            inside = 1,
            bestDistance = -1e9,
            nearestEdge = 0,
            i, v,
            circ2Pos, projection;
        for (i = 4; i--;) {

            // find the nearest face for center of circle    
            circ2Pos = c2.C;
            v = sub(circ2Pos, c1.X[i]);
            projection = dot(v, c1.N[i]);
            if (projection > 0) {

                // if the center of circle is outside of c1angle
                bestDistance = projection;
                nearestEdge = i;
                inside = 0;
                break;
            }

            if (projection > bestDistance) {
                bestDistance = projection;
                nearestEdge = i;
            }
        }
        let dis, normal;

        if (inside) {

            // the center of circle is inside of c1angle
            setInfo(collisionInfo, c2.B - bestDistance, c1.N[nearestEdge], sub(circ2Pos, mul(c1.N[nearestEdge], c2.B)));
        }
        else {

            // the center of circle is outside of c1angle
            // v1 is from left vertex of face to center of circle 
            // v2 is from left vertex of face to right vertex of face
            var
                v1 = sub(circ2Pos, c1.X[nearestEdge]),
                v2 = sub(c1.X[(nearestEdge + 1) % 4], c1.X[nearestEdge]),
                dotp = dot(v1, v2);
            if (dotp < 0) {

                // the center of circle is in corner region of X[nearestEdge]
                dis = mag(v1);

                // compare the distance with radium to decide collision
                if (dis > c2.B) {
                    return;
                }
                normal = normalize(v1);
                setInfo(collisionInfo, c2.B - dis, normal, add(circ2Pos, mul(normal, -c2.B)));
            }
            else {

                // the center of circle is in corner region of X[nearestEdge+1]
                // v1 is from right vertex of face to center of circle 
                // v2 is from right vertex of face to left vertex of face
                v1 = sub(circ2Pos, c1.X[(nearestEdge + 1) % 4]);
                v2 = mul(v2, -1);
                dotp = dot(v1, v2);
                if (dotp < 0) {
                    dis = mag(v1);

                    // compare the distance with radium to decide collision
                    if (dis > c2.B) {
                        return;
                    }
                    normal = normalize(v1);
                    setInfo(collisionInfo, c2.B - dis, normal, add(circ2Pos, mul(normal, -c2.B)));
                }

                else {

                    // the center of circle is in face region of face[nearestEdge]
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

export let resolveCollision = (s1: Shape, s2: Shape, collisionInfo: Collision) => {
    if (!s1.M && !s2.M) {
        return;
    }

    // correct positions
    var
        num = collisionInfo.D / (s1.M + s2.M) * .8, // .8 = poscorrectionrate = percentage of separation to project objects
        correctionAmount = mul(collisionInfo.N, num),
        n = collisionInfo.N;
    moveShape(s1, mul(correctionAmount, -s1.M));
    moveShape(s2, mul(correctionAmount, s2.M));

    // the direction of collisionInfo is always from s1 to s2
    // but the Mass is inversed, so start scale with s2 and end scale with s1
    var
        start = mul(collisionInfo.S, s2.M / (s1.M + s2.M)),
        end = mul(collisionInfo.E, s1.M / (s1.M + s2.M)),
        p = add(start, end),
        // r is vector from center of object to collision point
        r1 = sub(p, s1.C),
        r2 = sub(p, s2.C),

        // newV = V + v cross R
        v1 = add(s1.V, Vec2(-1 * s1.v * r1.y, s1.v * r1.x)),
        v2 = add(s2.V, Vec2(-1 * s2.v * r2.y, s2.v * r2.x)),
        relativeVelocity = sub(v2, v1),

        // Relative velocity in normal direction
        rVelocityInNormal = dot(relativeVelocity, n);

    // if objects moving apart ignore
    if (rVelocityInNormal > 0) {
        return;
    }

    // compute and apply response impulses for each object  
    let
        newRestituion = Math.min(s1.R, s2.R),
        newFriction = Math.min(s1.F, s2.F),

        // R cross N
        R1crossN = cross(r1, n),
        R2crossN = cross(r2, n),

        // Calc impulse scalar
        // the formula of jN can be found in http://www.myphysicslab.com/collision.html
        jN = (-(1 + newRestituion) * rVelocityInNormal) / (s1.M + s2.M + R1crossN * R1crossN * s1.I + R2crossN * R2crossN * s2.I),

        // impulse is in direction of normal ( from s1 to s2)
        impulse = mul(n, jN);

    // impulse = F dt = m * ?v
    // ?v = impulse / m
    s1.V = sub(s1.V, mul(impulse, s1.M));
    s2.V = add(s2.V, mul(impulse, s2.M));
    s1.v -= R1crossN * jN * s1.I;
    s2.v += R2crossN * jN * s2.I;
    let
        tangent = mul(normalize(sub(relativeVelocity, mul(n, dot(relativeVelocity, n)))), -1),
        R1crossT = cross(r1, tangent),
        R2crossT = cross(r2, tangent),
        jT = (-(1 + newRestituion) * dot(relativeVelocity, tangent) * newFriction) / (s1.M + s2.M + R1crossT * R1crossT * s1.I + R2crossT * R2crossT * s2.I);

    // friction should less than force in normal direction
    if (jT > jN) {
        jT = jN;
    }

    // impulse is from s1 to s2 (in opposite direction of velocity)
    impulse = mul(tangent, jT);
    s1.V = sub(s1.V, mul(impulse, s1.M));
    s2.V = add(s2.V, mul(impulse, s2.M));
    s1.v -= R1crossT * jT * s1.I;
    s2.v += R2crossT * jT * s2.I;
};


export function computeCollisions() {
    let i, j, k;

    for (k = 9; k--;) {
        for (i = objects.length; i--;) {
            for (j = objects.length; j-- > i;) {
                collideAB(objects[i], objects[j])
            }
        }
    }
}

export function computeCollisionsWith(car: Shape) {
    for (let o of objects) {
        if (o != car)
            collideAB(o, car)
    }
}


function collideAB(a: Shape, b: Shape) {
    // Test bounds
    if (boundTest(a, b)) {

        // Test collision
        if (testCollision(a, b, collisionInfo)) {

            // Make sure the normal is always from object[i] to object[j]
            if (dot(collisionInfo.N, sub(b.C, a.C)) < 0) {
                collisionInfo = {
                    D: collisionInfo.D,
                    N: mul(collisionInfo.N, -1),
                    S: collisionInfo.E,
                    E: collisionInfo.S
                };
            }

            // Resolve collision
            resolveCollision(a, b, collisionInfo);
        }
    }

}



export function updatePhysics() {
    for (let obj of objects) {
        // Update position/rotation
        obj.V = add(obj.V, mul(obj.A, 1 / 60));
        moveShape(obj, mul(obj.V, 1 / 60));
        obj.v += obj.a * 1 / 60;
        rotateShape(obj, obj.v * 1 / 60);
    }
}

// New circle
export let Circle = (center: Vec2, radius: number, mass?:number|undefined, friction = 0, restitution = 0) =>
    RigidShape(center, mass, friction, restitution, 0, radius);

// New rectangle
export let Rectangle = (center: Vec2, width: number, height: number, mass?:number, friction = 0, restitution = 0) =>
    RigidShape(center, mass, friction, restitution, 1, Math.hypot(width, height) / 2, width, height);

export function deleteObjects(){
    objects = objects.filter(o=>!o.del)
}
