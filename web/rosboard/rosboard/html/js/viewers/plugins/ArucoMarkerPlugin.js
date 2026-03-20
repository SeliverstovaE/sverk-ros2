"use strict";

// ArucoMarkerPlugin — renders ArUco markers with proper black/white pattern textures.
// Data format (aruco_det_loc/msg/MarkerArray):
//   { header, markers: [{ id, size, pose: {position, orientation}, corners: [Point32 x4] }] }
//
// Each marker is displayed as a textured plane with the ArUco grid pattern,
// positioned and oriented via pose, plus an ID label overhead.

class ArucoMarkerPlugin {
  /**
   * @param {THREE.Scene} scene
   * @param {jQuery} labelsOverlay - HTML overlay div for labels
   * @param {THREE.Camera} camera - for 3D→2D label projection
   */
  constructor(scene, labelsOverlay, camera) {
    this.scene = scene;
    this.labelsOverlay = labelsOverlay;
    this.camera = camera;

    this.group = new THREE.Group();
    this.scene.add(this.group);

    this._markerObjects = {};   // id → { smooth, mesh, border, axes, ... }
    this._textureCache = {};    // id → THREE.CanvasTexture
    this._labelElements = {};   // key → jQuery element
    this._lastMarkers = [];
    this._visible = true;
    this._smoothSpeed = 14;     // SmoothTransform speed for markers (camera data is noisy)
  }

  // ── ArUco Texture Generation ─────────────────────────────

  /**
   * Generate a 6×6 ArUco-style pattern (4×4 data grid + 1-cell black border).
   * Uses the binary representation of the marker ID for the inner cells.
   */
  _generateArucoTexture(id) {
    if (this._textureCache[id]) return this._textureCache[id];

    let gridSize = 6;    // 4×4 data + 1-cell border on each side
    let cellPx = 40;     // pixels per cell
    let canvasSize = gridSize * cellPx;

    let canvas = document.createElement('canvas');
    canvas.width = canvasSize;
    canvas.height = canvasSize;
    let ctx = canvas.getContext('2d');

    // White background
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, canvasSize, canvasSize);

    // Black border ring (outermost cells)
    ctx.fillStyle = '#000000';
    for (let i = 0; i < gridSize; i++) {
      // Top row
      ctx.fillRect(i * cellPx, 0, cellPx, cellPx);
      // Bottom row
      ctx.fillRect(i * cellPx, (gridSize - 1) * cellPx, cellPx, cellPx);
      // Left column
      ctx.fillRect(0, i * cellPx, cellPx, cellPx);
      // Right column
      ctx.fillRect((gridSize - 1) * cellPx, i * cellPx, cellPx, cellPx);
    }

    // Inner 4×4 data cells — deterministic pattern from marker ID
    let bits = this._idToBitPattern(id);
    for (let row = 0; row < 4; row++) {
      for (let col = 0; col < 4; col++) {
        ctx.fillStyle = bits[row * 4 + col] ? '#000000' : '#ffffff';
        ctx.fillRect((col + 1) * cellPx, (row + 1) * cellPx, cellPx, cellPx);
      }
    }

    // ID number in the center (small, semi-transparent, for quick identification)
    ctx.fillStyle = 'rgba(0, 200, 100, 0.7)';
    ctx.font = 'bold ' + (cellPx * 1.2) + 'px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.strokeStyle = 'rgba(0, 0, 0, 0.5)';
    ctx.lineWidth = 3;
    ctx.strokeText(String(id), canvasSize / 2, canvasSize / 2);
    ctx.fillText(String(id), canvasSize / 2, canvasSize / 2);

    let texture = new THREE.CanvasTexture(canvas);
    texture.magFilter = THREE.NearestFilter;
    texture.minFilter = THREE.NearestFilter;
    texture.generateMipmaps = false;

    this._textureCache[id] = texture;
    return texture;
  }

  /**
   * Convert marker ID to a 16-bit pattern for the 4×4 inner grid.
   * Uses a simple hash-like mixing to produce a distinct pattern per ID.
   */
  _idToBitPattern(id) {
    // Mix the ID bits to create a visually distinct pattern for each ID.
    // Real ArUco dictionaries use Hamming-code-based patterns;
    // this is a visual approximation that ensures different IDs look different.
    let v = ((id + 1) * 2654435761) >>> 0;  // Knuth multiplicative hash
    let bits = [];
    for (let i = 0; i < 16; i++) {
      bits.push((v >> i) & 1);
    }
    // Ensure at least some black and some white cells for visual variety
    let sum = bits.reduce((a, b) => a + b, 0);
    if (sum < 3) { bits[0] = 1; bits[5] = 1; bits[10] = 1; }
    if (sum > 13) { bits[3] = 0; bits[7] = 0; bits[12] = 0; }
    return bits;
  }

  // ── Marker mesh creation ───────────────────────────────────

  _getOrCreateMarker(id, size) {
    if (this._markerObjects[id]) return this._markerObjects[id];

    let markerSize = size || 0.15;
    let smooth = new SmoothTransform(this.group, { speed: this._smoothSpeed, snapDistance: 1.5 });

    // ArUco textured plane
    let texture = this._generateArucoTexture(id);
    let geo = new THREE.PlaneGeometry(markerSize, markerSize);
    let mat = new THREE.MeshBasicMaterial({
      map: texture,
      side: THREE.DoubleSide,
    });
    let mesh = new THREE.Mesh(geo, mat);
    smooth.group.add(mesh);

    // Green border outline
    let borderPoints = [
      new THREE.Vector3(-markerSize / 2, -markerSize / 2, 0),
      new THREE.Vector3( markerSize / 2, -markerSize / 2, 0),
      new THREE.Vector3( markerSize / 2,  markerSize / 2, 0),
      new THREE.Vector3(-markerSize / 2,  markerSize / 2, 0),
    ];
    let borderGeo = new THREE.BufferGeometry().setFromPoints(borderPoints);
    let borderMat = new THREE.LineBasicMaterial({ color: 0x44ff88, linewidth: 2 });
    let border = new THREE.LineLoop(borderGeo, borderMat);
    smooth.group.add(border);

    // Mini axes at marker center (shows orientation)
    let axes = new THREE.AxesHelper(markerSize * 0.6);
    smooth.group.add(axes);

    let obj = { smooth, mesh, border, axes, geo, mat, texture, size: markerSize };
    this._markerObjects[id] = obj;
    return obj;
  }

  // ── Update from topic data ─────────────────────────────────

  /**
   * @param {Array} markers - array of { id, size, pose, corners }
   */
  updateMarkers(markers) {
    if (!Array.isArray(markers)) return;
    this._lastMarkers = markers;

    let activeIds = new Set();

    for (let i = 0; i < markers.length; i++) {
      let m = markers[i];
      if (m.id == null) continue;
      activeIds.add(m.id);

      let obj = this._getOrCreateMarker(m.id, m.size);

      // Set target pose (SmoothTransform will interpolate in updateSmooth)
      if (m.pose) {
        obj.smooth.setTarget(m.pose.position, m.pose.orientation);
      }

      obj.smooth.group.visible = this._visible;
    }

    // Hide markers that are no longer present
    for (let id in this._markerObjects) {
      if (!activeIds.has(Number(id)) && !activeIds.has(id)) {
        this._markerObjects[id].smooth.group.visible = false;
      }
    }
  }

  // ── Labels (HTML overlay) ──────────────────────────────────

  updateLabels() {
    if (!this.labelsOverlay || !this._visible) {
      for (let key in this._labelElements) {
        this._labelElements[key].css("display", "none");
      }
      return;
    }

    let activeKeys = new Set();

    for (let i = 0; i < this._lastMarkers.length; i++) {
      let m = this._lastMarkers[i];
      if (m.id == null || !m.pose || !m.pose.position) continue;

      let key = "aruco_" + m.id;
      activeKeys.add(key);

      let p = m.pose.position;
      // Label slightly above the marker
      let screenPos = this._project3DTo2D(p.x || 0, p.y || 0, (p.z || 0) + 0.12);
      this._setLabel(key, m.id, screenPos);
    }

    // Remove unused labels
    for (let key in this._labelElements) {
      if (!activeKeys.has(key)) {
        this._labelElements[key].remove();
        delete this._labelElements[key];
      }
    }
  }

  _project3DTo2D(x, y, z) {
    let v = new THREE.Vector3(x, y, z);
    v.project(this.camera);
    if (Math.abs(v.x) > 2 || Math.abs(v.y) > 2 || v.z > 1) return null;
    return {
      x: (v.x * 0.5 + 0.5) * 100,
      y: (-v.y * 0.5 + 0.5) * 100,
    };
  }

  _setLabel(key, id, screenPos) {
    if (!screenPos) {
      if (this._labelElements[key]) this._labelElements[key].css("display", "none");
      return;
    }
    if (!this._labelElements[key]) {
      this._labelElements[key] = $('<div></div>').css({
        "position": "absolute",
        "font-size": "8px",
        "font-family": "'JetBrains Mono', monospace",
        "color": "rgba(180,190,200,0.6)",
        "text-shadow": "0 0 2px rgba(0,0,0,0.8)",
        "white-space": "nowrap",
        "pointer-events": "none",
        "transform": "translate(4px, -50%)",
      }).appendTo(this.labelsOverlay);
    }
    this._labelElements[key].text("id:" + id).css({
      "display": "",
      "left": screenPos.x + "%",
      "top": screenPos.y + "%",
    });
  }

  // ── Smooth update (call every frame from render loop) ─────

  /** Interpolate all markers towards targets. Call from render loop. */
  updateSmooth(dt) {
    for (let id in this._markerObjects) {
      this._markerObjects[id].smooth.update(dt);
    }
  }

  // ── Public API ─────────────────────────────────────────────

  setVisible(visible) {
    this._visible = !!visible;
    this.group.visible = this._visible;
    if (!this._visible) {
      for (let key in this._labelElements) {
        this._labelElements[key].css("display", "none");
      }
    }
  }

  destroy() {
    // Remove 3D objects
    for (let id in this._markerObjects) {
      let obj = this._markerObjects[id];
      if (obj.geo) obj.geo.dispose();
      if (obj.mat) obj.mat.dispose();
      if (obj.texture) obj.texture.dispose();
      if (obj.border) {
        obj.border.geometry.dispose();
        obj.border.material.dispose();
      }
      if (obj.axes) obj.axes.dispose();
      obj.smooth.destroy();
    }
    this._markerObjects = {};

    // Dispose cached textures
    for (let id in this._textureCache) {
      this._textureCache[id].dispose();
    }
    this._textureCache = {};

    this.scene.remove(this.group);

    // Remove labels
    for (let key in this._labelElements) {
      this._labelElements[key].remove();
    }
    this._labelElements = {};
  }
}
