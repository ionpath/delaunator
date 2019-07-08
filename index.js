"use strict";

Object.defineProperty(exports, "__esModule", {
  value: true
});
exports["default"] = void 0;

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

function _defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } }

function _createClass(Constructor, protoProps, staticProps) { if (protoProps) _defineProperties(Constructor.prototype, protoProps); if (staticProps) _defineProperties(Constructor, staticProps); return Constructor; }

var EPSILON = Math.pow(2, -52);
var EDGE_STACK = new Uint32Array(512);

var Delaunator =
/*#__PURE__*/
function () {
  _createClass(Delaunator, null, [{
    key: "from",
    value: function from(points) {
      var getX = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : defaultGetX;
      var getY = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : defaultGetY;
      var n = points.length;
      var coords = new Float64Array(n * 2);

      for (var i = 0; i < n; i++) {
        var p = points[i];
        coords[2 * i] = getX(p);
        coords[2 * i + 1] = getY(p);
      }

      return new Delaunator(coords);
    }
  }]);

  function Delaunator(coords) {
    _classCallCheck(this, Delaunator);

    var n = coords.length >> 1;
    if (n > 0 && typeof coords[0] !== 'number') throw new Error('Expected coords to contain numbers.');
    this.coords = coords; // arrays that will store the triangulation graph

    var maxTriangles = Math.max(2 * n - 5, 0);
    this._triangles = new Uint32Array(maxTriangles * 3);
    this._halfedges = new Int32Array(maxTriangles * 3); // temporary arrays for tracking the edges of the advancing convex hull

    this._hashSize = Math.ceil(Math.sqrt(n));
    this._hullPrev = new Uint32Array(n); // edge to prev edge

    this._hullNext = new Uint32Array(n); // edge to next edge

    this._hullTri = new Uint32Array(n); // edge to adjacent triangle

    this._hullHash = new Int32Array(this._hashSize).fill(-1); // angular edge hash
    // temporary arrays for sorting points

    this._ids = new Uint32Array(n);
    this._dists = new Float64Array(n);
    this.update();
  }

  _createClass(Delaunator, [{
    key: "update",
    value: function update() {
      var coords = this.coords,
          hullPrev = this._hullPrev,
          hullNext = this._hullNext,
          hullTri = this._hullTri,
          hullHash = this._hullHash;
      var n = coords.length >> 1; // populate an array of point indices; calculate input data bbox

      var minX = Infinity;
      var minY = Infinity;
      var maxX = -Infinity;
      var maxY = -Infinity;

      for (var i = 0; i < n; i++) {
        var x = coords[2 * i];
        var y = coords[2 * i + 1];
        if (x < minX) minX = x;
        if (y < minY) minY = y;
        if (x > maxX) maxX = x;
        if (y > maxY) maxY = y;
        this._ids[i] = i;
      }

      var cx = (minX + maxX) / 2;
      var cy = (minY + maxY) / 2;
      var minDist = Infinity;
      var i0, i1, i2; // pick a seed point close to the center

      for (var _i = 0; _i < n; _i++) {
        var d = dist(cx, cy, coords[2 * _i], coords[2 * _i + 1]);

        if (d < minDist) {
          i0 = _i;
          minDist = d;
        }
      }

      var i0x = coords[2 * i0];
      var i0y = coords[2 * i0 + 1];
      minDist = Infinity; // find the point closest to the seed

      for (var _i2 = 0; _i2 < n; _i2++) {
        if (_i2 === i0) continue;

        var _d = dist(i0x, i0y, coords[2 * _i2], coords[2 * _i2 + 1]);

        if (_d < minDist && _d > 0) {
          i1 = _i2;
          minDist = _d;
        }
      }

      var i1x = coords[2 * i1];
      var i1y = coords[2 * i1 + 1];
      var minRadius = Infinity; // find the third point which forms the smallest circumcircle with the first two

      for (var _i3 = 0; _i3 < n; _i3++) {
        if (_i3 === i0 || _i3 === i1) continue;
        var r = circumradius(i0x, i0y, i1x, i1y, coords[2 * _i3], coords[2 * _i3 + 1]);

        if (r < minRadius) {
          i2 = _i3;
          minRadius = r;
        }
      }

      var i2x = coords[2 * i2];
      var i2y = coords[2 * i2 + 1];

      if (minRadius === Infinity) {
        // order collinear points by dx (or dy if all x are identical)
        // and return the list as a hull
        for (var _i4 = 0; _i4 < n; _i4++) {
          this._dists[_i4] = coords[2 * _i4] - coords[0] || coords[2 * _i4 + 1] - coords[1];
        }

        quicksort(this._ids, this._dists, 0, n - 1);
        var hull = new Uint32Array(n);
        var j = 0;

        for (var _i5 = 0, d0 = -Infinity; _i5 < n; _i5++) {
          var id = this._ids[_i5];

          if (this._dists[id] > d0) {
            hull[j++] = id;
            d0 = this._dists[id];
          }
        }

        this.hull = hull.subarray(0, j);
        this.triangles = new Uint32Array(0);
        this.halfedges = new Uint32Array(0);
        return;
      } // swap the order of the seed points for counter-clockwise orientation


      if (orient(i0x, i0y, i1x, i1y, i2x, i2y)) {
        var _i6 = i1;
        var _x = i1x;
        var _y = i1y;
        i1 = i2;
        i1x = i2x;
        i1y = i2y;
        i2 = _i6;
        i2x = _x;
        i2y = _y;
      }

      var center = circumcenter(i0x, i0y, i1x, i1y, i2x, i2y);
      this._cx = center.x;
      this._cy = center.y;

      for (var _i7 = 0; _i7 < n; _i7++) {
        this._dists[_i7] = dist(coords[2 * _i7], coords[2 * _i7 + 1], center.x, center.y);
      } // sort the points by distance from the seed triangle circumcenter


      quicksort(this._ids, this._dists, 0, n - 1); // set up the seed triangle as the starting hull

      this._hullStart = i0;
      var hullSize = 3;
      hullNext[i0] = hullPrev[i2] = i1;
      hullNext[i1] = hullPrev[i0] = i2;
      hullNext[i2] = hullPrev[i1] = i0;
      hullTri[i0] = 0;
      hullTri[i1] = 1;
      hullTri[i2] = 2;
      hullHash.fill(-1);
      hullHash[this._hashKey(i0x, i0y)] = i0;
      hullHash[this._hashKey(i1x, i1y)] = i1;
      hullHash[this._hashKey(i2x, i2y)] = i2;
      this.trianglesLen = 0;

      this._addTriangle(i0, i1, i2, -1, -1, -1);

      for (var k = 0, xp, yp; k < this._ids.length; k++) {
        var _i8 = this._ids[k];
        var _x2 = coords[2 * _i8];
        var _y2 = coords[2 * _i8 + 1]; // skip near-duplicate points

        if (k > 0 && Math.abs(_x2 - xp) <= EPSILON && Math.abs(_y2 - yp) <= EPSILON) continue;
        xp = _x2;
        yp = _y2; // skip seed triangle points

        if (_i8 === i0 || _i8 === i1 || _i8 === i2) continue; // find a visible edge on the convex hull using edge hash

        var start = 0;

        for (var _j = 0, key = this._hashKey(_x2, _y2); _j < this._hashSize; _j++) {
          start = hullHash[(key + _j) % this._hashSize];
          if (start !== -1 && start !== hullNext[start]) break;
        }

        start = hullPrev[start];
        var e = start,
            q = void 0;

        while (q = hullNext[e], !orient(_x2, _y2, coords[2 * e], coords[2 * e + 1], coords[2 * q], coords[2 * q + 1])) {
          e = q;

          if (e === start) {
            e = -1;
            break;
          }
        }

        if (e === -1) continue; // likely a near-duplicate point; skip it
        // add the first triangle from the point

        var t = this._addTriangle(e, _i8, hullNext[e], -1, -1, hullTri[e]); // recursively flip triangles from the point until they satisfy the Delaunay condition


        hullTri[_i8] = this._legalize(t + 2);
        hullTri[e] = t; // keep track of boundary triangles on the hull

        hullSize++; // walk forward through the hull, adding more triangles and flipping recursively

        var _n = hullNext[e];

        while (q = hullNext[_n], orient(_x2, _y2, coords[2 * _n], coords[2 * _n + 1], coords[2 * q], coords[2 * q + 1])) {
          t = this._addTriangle(_n, _i8, q, hullTri[_i8], -1, hullTri[_n]);
          hullTri[_i8] = this._legalize(t + 2);
          hullNext[_n] = _n; // mark as removed

          hullSize--;
          _n = q;
        } // walk backward from the other side, adding more triangles and flipping


        if (e === start) {
          while (q = hullPrev[e], orient(_x2, _y2, coords[2 * q], coords[2 * q + 1], coords[2 * e], coords[2 * e + 1])) {
            t = this._addTriangle(q, _i8, e, -1, hullTri[e], hullTri[q]);

            this._legalize(t + 2);

            hullTri[q] = t;
            hullNext[e] = e; // mark as removed

            hullSize--;
            e = q;
          }
        } // update the hull indices


        this._hullStart = hullPrev[_i8] = e;
        hullNext[e] = hullPrev[_n] = _i8;
        hullNext[_i8] = _n; // save the two new edges in the hash table

        hullHash[this._hashKey(_x2, _y2)] = _i8;
        hullHash[this._hashKey(coords[2 * e], coords[2 * e + 1])] = e;
      }

      this.hull = new Uint32Array(hullSize);

      for (var _i9 = 0, _e = this._hullStart; _i9 < hullSize; _i9++) {
        this.hull[_i9] = _e;
        _e = hullNext[_e];
      } // trim typed triangle mesh arrays


      this.triangles = this._triangles.subarray(0, this.trianglesLen);
      this.halfedges = this._halfedges.subarray(0, this.trianglesLen);
    }
  }, {
    key: "_hashKey",
    value: function _hashKey(x, y) {
      return Math.floor(pseudoAngle(x - this._cx, y - this._cy) * this._hashSize) % this._hashSize;
    }
  }, {
    key: "_legalize",
    value: function _legalize(a) {
      var triangles = this._triangles,
          halfedges = this._halfedges,
          coords = this.coords;
      var i = 0;
      var ar = 0; // recursion eliminated with a fixed-size stack

      while (true) {
        var b = halfedges[a];
        /* if the pair of triangles doesn't satisfy the Delaunay condition
         * (p1 is inside the circumcircle of [p0, pl, pr]), flip them,
         * then do the same check/flip recursively for the new pair of triangles
         *
         *           pl                    pl
         *          /||\                  /  \
         *       al/ || \bl            al/    \a
         *        /  ||  \              /      \
         *       /  a||b  \    flip    /___ar___\
         *     p0\   ||   /p1   =>   p0\---bl---/p1
         *        \  ||  /              \      /
         *       ar\ || /br             b\    /br
         *          \||/                  \  /
         *           pr                    pr
         */

        var a0 = a - a % 3;
        ar = a0 + (a + 2) % 3;

        if (b === -1) {
          // convex hull edge
          if (i === 0) break;
          a = EDGE_STACK[--i];
          continue;
        }

        var b0 = b - b % 3;
        var al = a0 + (a + 1) % 3;
        var bl = b0 + (b + 2) % 3;
        var p0 = triangles[ar];
        var pr = triangles[a];
        var pl = triangles[al];
        var p1 = triangles[bl];
        var illegal = inCircle(coords[2 * p0], coords[2 * p0 + 1], coords[2 * pr], coords[2 * pr + 1], coords[2 * pl], coords[2 * pl + 1], coords[2 * p1], coords[2 * p1 + 1]);

        if (illegal) {
          triangles[a] = p1;
          triangles[b] = p0;
          var hbl = halfedges[bl]; // edge swapped on the other side of the hull (rare); fix the halfedge reference

          if (hbl === -1) {
            var e = this._hullStart;

            do {
              if (this._hullTri[e] === bl) {
                this._hullTri[e] = a;
                break;
              }

              e = this._hullPrev[e];
            } while (e !== this._hullStart);
          }

          this._link(a, hbl);

          this._link(b, halfedges[ar]);

          this._link(ar, bl);

          var br = b0 + (b + 1) % 3; // don't worry about hitting the cap: it can only happen on extremely degenerate input

          if (i < EDGE_STACK.length) {
            EDGE_STACK[i++] = br;
          }
        } else {
          if (i === 0) break;
          a = EDGE_STACK[--i];
        }
      }

      return ar;
    }
  }, {
    key: "_link",
    value: function _link(a, b) {
      this._halfedges[a] = b;
      if (b !== -1) this._halfedges[b] = a;
    } // add a new triangle given vertex indices and adjacent half-edge ids

  }, {
    key: "_addTriangle",
    value: function _addTriangle(i0, i1, i2, a, b, c) {
      var t = this.trianglesLen;
      this._triangles[t] = i0;
      this._triangles[t + 1] = i1;
      this._triangles[t + 2] = i2;

      this._link(t, a);

      this._link(t + 1, b);

      this._link(t + 2, c);

      this.trianglesLen += 3;
      return t;
    }
  }]);

  return Delaunator;
}(); // monotonically increases with real angle, but doesn't need expensive trigonometry


exports["default"] = Delaunator;

function pseudoAngle(dx, dy) {
  var p = dx / (Math.abs(dx) + Math.abs(dy));
  return (dy > 0 ? 3 - p : 1 + p) / 4; // [0..1]
}

function dist(ax, ay, bx, by) {
  var dx = ax - bx;
  var dy = ay - by;
  return dx * dx + dy * dy;
}

function orient(px, py, qx, qy, rx, ry) {
  return (qy - py) * (rx - qx) - (qx - px) * (ry - qy) < 0;
}

function inCircle(ax, ay, bx, by, cx, cy, px, py) {
  var dx = ax - px;
  var dy = ay - py;
  var ex = bx - px;
  var ey = by - py;
  var fx = cx - px;
  var fy = cy - py;
  var ap = dx * dx + dy * dy;
  var bp = ex * ex + ey * ey;
  var cp = fx * fx + fy * fy;
  return dx * (ey * cp - bp * fy) - dy * (ex * cp - bp * fx) + ap * (ex * fy - ey * fx) < 0;
}

function circumradius(ax, ay, bx, by, cx, cy) {
  var dx = bx - ax;
  var dy = by - ay;
  var ex = cx - ax;
  var ey = cy - ay;
  var bl = dx * dx + dy * dy;
  var cl = ex * ex + ey * ey;
  var d = 0.5 / (dx * ey - dy * ex);
  var x = (ey * bl - dy * cl) * d;
  var y = (dx * cl - ex * bl) * d;
  return x * x + y * y;
}

function circumcenter(ax, ay, bx, by, cx, cy) {
  var dx = bx - ax;
  var dy = by - ay;
  var ex = cx - ax;
  var ey = cy - ay;
  var bl = dx * dx + dy * dy;
  var cl = ex * ex + ey * ey;
  var d = 0.5 / (dx * ey - dy * ex);
  var x = ax + (ey * bl - dy * cl) * d;
  var y = ay + (dx * cl - ex * bl) * d;
  return {
    x: x,
    y: y
  };
}

function quicksort(ids, dists, left, right) {
  if (right - left <= 20) {
    for (var i = left + 1; i <= right; i++) {
      var temp = ids[i];
      var tempDist = dists[temp];
      var j = i - 1;

      while (j >= left && dists[ids[j]] > tempDist) {
        ids[j + 1] = ids[j--];
      }

      ids[j + 1] = temp;
    }
  } else {
    var median = left + right >> 1;

    var _i10 = left + 1;

    var _j2 = right;
    swap(ids, median, _i10);
    if (dists[ids[left]] > dists[ids[right]]) swap(ids, left, right);
    if (dists[ids[_i10]] > dists[ids[right]]) swap(ids, _i10, right);
    if (dists[ids[left]] > dists[ids[_i10]]) swap(ids, left, _i10);
    var _temp = ids[_i10];
    var _tempDist = dists[_temp];

    while (true) {
      do {
        _i10++;
      } while (dists[ids[_i10]] < _tempDist);

      do {
        _j2--;
      } while (dists[ids[_j2]] > _tempDist);

      if (_j2 < _i10) break;
      swap(ids, _i10, _j2);
    }

    ids[left + 1] = ids[_j2];
    ids[_j2] = _temp;

    if (right - _i10 + 1 >= _j2 - left) {
      quicksort(ids, dists, _i10, right);
      quicksort(ids, dists, left, _j2 - 1);
    } else {
      quicksort(ids, dists, left, _j2 - 1);
      quicksort(ids, dists, _i10, right);
    }
  }
}

function swap(arr, i, j) {
  var tmp = arr[i];
  arr[i] = arr[j];
  arr[j] = tmp;
}

function defaultGetX(p) {
  return p[0];
}

function defaultGetY(p) {
  return p[1];
}
