"use strict";

// RobotModelPlugin — loads an FBX 3D model and attaches it to a TF frame.
// Usage:
//   let plugin = new RobotModelPlugin(scene, { modelUrl: '/models/obrik-sim2.fbx' });
//   plugin.setTransform(position, quaternion); // call on each TF update
//   plugin.setVisible(false);
//   plugin.destroy();

class RobotModelPlugin {
  /**
   * @param {THREE.Scene} scene
   * @param {Object} options
   * @param {string} options.modelUrl - URL of the FBX model
   * @param {number} [options.targetSize=0.35] - desired bounding box size (meters)
   * @param {Function} [options.onLoad] - callback after successful load
   * @param {Function} [options.onError] - callback on load failure
   */
  constructor(scene, options = {}) {
    this.scene = scene;
    this.modelUrl = options.modelUrl || '/models/obrik-sim2.fbx';
    this.targetSize = options.targetSize || 0.35;
    this.onLoadCallback = options.onLoad || null;
    this.onErrorCallback = options.onError || null;

    // Faster catch-up + snap on big teleports to avoid visible lag
    this._smooth = new SmoothTransform(scene, { speed: 25, snapDistance: 0.75 });
    this.group = this._smooth.group;

    this._model = null;
    this._pivot = null;
    this._correction = null;
    this._fallbackMarker = null;
    this._loaded = false;

    this._createFallbackMarker();
    this._loadModel();
  }

  // ── Fallback marker (shown while model loads) ──────────────

  _createFallbackMarker() {
    let geo = new THREE.SphereGeometry(0.08, 16, 12);
    let mat = new THREE.MeshPhongMaterial({
      color: 0x00ffaa, emissive: 0x006644, transparent: true, opacity: 0.6,
    });
    this._fallbackMarker = new THREE.Mesh(geo, mat);
    this.group.add(this._fallbackMarker);
  }

  _removeFallbackMarker() {
    if (!this._fallbackMarker) return;
    this.group.remove(this._fallbackMarker);
    this._fallbackMarker.geometry.dispose();
    this._fallbackMarker.material.dispose();
    this._fallbackMarker = null;
  }

  // ── FBX Loading ────────────────────────────────────────────

  _loadModel() {
    if (typeof THREE === 'undefined' || typeof THREE.FBXLoader === 'undefined') {
      console.warn('[RobotModelPlugin] THREE.FBXLoader not available');
      if (this.onErrorCallback) this.onErrorCallback('FBXLoader not available');
      return;
    }

    let that = this;
    let loader = new THREE.FBXLoader();

    loader.load(this.modelUrl, (object) => {
      // FBXLoader converts to Three.js Y-up. Our scene is Z-up (ROS).
      // Correction: rotate +90deg around X → Y-up becomes Z-up.
      let correction = new THREE.Group();
      correction.rotation.x = Math.PI / 2;
      correction.add(object);

      // Auto-scale to targetSize
      correction.updateMatrixWorld(true);
      let box = new THREE.Box3().setFromObject(correction);
      let size = new THREE.Vector3();
      box.getSize(size);
      let maxDim = Math.max(size.x, size.y, size.z);
      let s = maxDim > 0 ? that.targetSize / maxDim : 1.0;

      let pivot = new THREE.Group();
      pivot.add(correction);
      pivot.scale.set(s, s, s);

      // Center on bounding box
      pivot.updateMatrixWorld(true);
      box.setFromObject(pivot);
      let center = new THREE.Vector3();
      box.getCenter(center);
      correction.position.set(-center.x / s, -center.y / s, -center.z / s);

      // Preserve original FBX materials, ensure double-sided
      object.traverse((child) => {
        if (child.isMesh && child.material) {
          let mats = Array.isArray(child.material) ? child.material : [child.material];
          mats.forEach((m) => { m.side = THREE.DoubleSide; });
        }
      });

      // Swap fallback for real model
      that._removeFallbackMarker();
      that._model = object;
      that._pivot = pivot;
      that._correction = correction;
      that._loaded = true;
      that.group.add(pivot);

      console.log('[RobotModelPlugin] Model loaded. bbox:',
        size.x.toFixed(1), 'x', size.y.toFixed(1), 'x', size.z.toFixed(1),
        ', scale:', s.toFixed(6));

      if (that.onLoadCallback) that.onLoadCallback(object);
    }, (xhr) => {
      if (xhr.total) {
        console.log('[RobotModelPlugin] Loading:', Math.round(xhr.loaded / xhr.total * 100) + '%');
      }
    }, (error) => {
      console.warn('[RobotModelPlugin] FBX load failed:', error.message || error);
      if (that.onErrorCallback) that.onErrorCallback(error);
    });
  }

  // ── Public API ─────────────────────────────────────────────

  /** Update target position and rotation (called each TF update for the attached frame). */
  setTransform(position, quaternion) {
    this._smooth.setTarget(position, quaternion);
  }

  /** Call every frame from the render loop to smoothly interpolate towards target. */
  updateSmooth(dt) {
    this._smooth.update(dt);
  }

  /** Show or hide the model. */
  setVisible(visible) {
    this.group.visible = !!visible;
  }

  /** Whether the real model has finished loading. */
  isLoaded() {
    return this._loaded;
  }

  /** Cleanup. */
  destroy() {
    this._removeFallbackMarker();
    if (this._pivot) {
      this.group.remove(this._pivot);
    }
    this._smooth.destroy();
    // Note: Three.js geometry/material disposal for loaded FBX
    // is complex; the scene GC handles most of it.
  }
}
