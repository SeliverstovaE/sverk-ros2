"use strict";

// SmoothTransform — smoothly interpolates a THREE.Group towards a target pose.
//
// Usage:
//   let st = new SmoothTransform(scene, { speed: 10 });
//   st.setTarget(position, quaternion);  // call when new data arrives
//   st.update(dt);                       // call every frame in render loop
//   st.group                             // the THREE.Group to attach children to
//   st.destroy();

class SmoothTransform {
  /**
   * @param {THREE.Object3D} parent - parent to add the group to (scene or another group)
   * @param {Object} [options]
   * @param {number} [options.speed=10] - interpolation speed (higher = faster catch-up)
   * @param {boolean} [options.immediate=false] - if true, first setTarget snaps instantly
   */
  constructor(parent, options = {}) {
    this.group = new THREE.Group();
    this._parent = parent;
    this._parent.add(this.group);

    this._speed = options.speed || 10;
    this._immediate = options.immediate !== false; // snap to first target by default
    this._snapDistance = options.snapDistance == null ? null : options.snapDistance;
    this._hasTarget = false;

    this._targetPos = new THREE.Vector3();
    this._targetQuat = new THREE.Quaternion();
  }

  setTarget(position, quaternion) {
    if (position) {
      if (position.isVector3) this._targetPos.copy(position);
      else this._targetPos.set(position.x || 0, position.y || 0, position.z || 0);
    }
    if (quaternion) {
      if (quaternion.isQuaternion) this._targetQuat.copy(quaternion);
      else this._targetQuat.set(
        quaternion.x || 0, quaternion.y || 0, quaternion.z || 0,
        quaternion.w == null ? 1 : quaternion.w
      );
      this._targetQuat.normalize();
    }

    // Snap instantly on first target (no weird lerp from origin)
    if (!this._hasTarget && this._immediate) {
      this.group.position.copy(this._targetPos);
      this.group.quaternion.copy(this._targetQuat);
    }
    this._hasTarget = true;
  }

  /**
   * Call every frame. Smoothly interpolates towards the target.
   * @param {number} dt - delta time in seconds
   */
  update(dt) {
    if (!this._hasTarget) return;
    if (this._snapDistance != null) {
      let d2 = this.group.position.distanceToSquared(this._targetPos);
      if (d2 > this._snapDistance * this._snapDistance) {
        this.snap();
        return;
      }
    }
    let t = Math.min(1.0, this._speed * dt);
    this.group.position.lerp(this._targetPos, t);
    this.group.quaternion.slerp(this._targetQuat, t);
  }

  /** Snap immediately to target (no interpolation). */
  snap() {
    if (!this._hasTarget) return;
    this.group.position.copy(this._targetPos);
    this.group.quaternion.copy(this._targetQuat);
  }

  /** Change interpolation speed. */
  setSpeed(speed) {
    this._speed = speed;
  }

  /** Remove from parent. */
  destroy() {
    this._parent.remove(this.group);
  }
}
