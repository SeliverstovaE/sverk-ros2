"use strict";

// TFUtils — utility for transforming poses between TF frames.
//
// Usage:
//   // tfTree — result of TFViewer._buildTransformTree(rootFrameId)
//   let worldPose = TFUtils.transformPose(pose, frameId, rootFrameId, tfTree);
//   let worldMarkers = TFUtils.transformMarkers(markers, frameId, rootFrameId, tfTree);
//   let { position, quaternion } = TFUtils.getFrameWorldTransform(frameId, rootFrameId, tfTree);

var TFUtils = {

  /**
   * Get the world-space position and quaternion of a TF frame.
   * Returns null if the frame is not found in the tree.
   * If frameId === rootFrameId, returns identity (origin).
   *
   * @param {string} frameId - source frame
   * @param {string} rootFrameId - root frame of the TF tree
   * @param {Object} tfTree - frame→{position, quaternion} map from _buildTransformTree
   * @returns {{ position: THREE.Vector3, quaternion: THREE.Quaternion } | null}
   */
  getFrameWorldTransform: function(frameId, rootFrameId, tfTree) {
    if (!frameId) return null;
    if (frameId === rootFrameId) return { position: new THREE.Vector3(), quaternion: new THREE.Quaternion() };
    if (!tfTree || !tfTree[frameId]) return null;
    return {
      position: tfTree[frameId].position.clone(),
      quaternion: tfTree[frameId].quaternion.clone(),
    };
  },

  /**
   * Transform a single pose { position: {x,y,z}, orientation: {x,y,z,w} }
   * from frameId into rootFrameId using the TF tree.
   * Returns a new pose object (does not mutate input).
   *
   * @param {Object} pose - { position: {x,y,z}, orientation: {x,y,z,w} }
   * @param {string} frameId - frame the pose is in
   * @param {string} rootFrameId
   * @param {Object} tfTree
   * @returns {Object} transformed pose, or original if no transform needed
   */
  transformPose: function(pose, frameId, rootFrameId, tfTree) {
    if (!pose) return pose;

    let ft = TFUtils.getFrameWorldTransform(frameId, rootFrameId, tfTree);
    if (!ft) return pose; // frame unknown, pass through

    let fp = ft.position;
    let fq = ft.quaternion;

    // Identity check — skip math if at origin with no rotation
    if (fp.lengthSq() < 1e-12 && Math.abs(1 - fq.w) < 1e-6) return pose;

    let lp = pose.position || {};
    let lo = pose.orientation || {};

    let localPos = new THREE.Vector3(lp.x || 0, lp.y || 0, lp.z || 0);
    let localQuat = new THREE.Quaternion(
      lo.x || 0, lo.y || 0, lo.z || 0, lo.w == null ? 1 : lo.w
    ).normalize();

    let worldPos = localPos.applyQuaternion(fq).add(fp);
    let worldQuat = fq.clone().multiply(localQuat);

    return {
      position: { x: worldPos.x, y: worldPos.y, z: worldPos.z },
      orientation: { x: worldQuat.x, y: worldQuat.y, z: worldQuat.z, w: worldQuat.w },
    };
  },

  /**
   * Transform an array of markers (each with .pose and optionally .corners)
   * from frameId into rootFrameId. Returns a new array.
   *
   * @param {Array} markers - [{ id, size, pose, corners }, ...]
   * @param {string} frameId
   * @param {string} rootFrameId
   * @param {Object} tfTree
   * @returns {Array} transformed markers
   */
  transformMarkers: function(markers, frameId, rootFrameId, tfTree) {
    if (!markers || !markers.length) return markers;

    let ft = TFUtils.getFrameWorldTransform(frameId, rootFrameId, tfTree);
    if (!ft) return markers;

    let fp = ft.position;
    let fq = ft.quaternion;
    let isIdentity = fp.lengthSq() < 1e-12 && Math.abs(1 - fq.w) < 1e-6;
    if (isIdentity) return markers;

    let result = [];
    for (let i = 0; i < markers.length; i++) {
      let m = markers[i];
      let tm = { id: m.id, size: m.size };

      // Transform pose (using pre-computed fp/fq, not re-looking up)
      if (m.pose) {
        let lp = m.pose.position || {};
        let lo = m.pose.orientation || {};
        let localPos = new THREE.Vector3(lp.x || 0, lp.y || 0, lp.z || 0);
        let localQuat = new THREE.Quaternion(
          lo.x || 0, lo.y || 0, lo.z || 0, lo.w == null ? 1 : lo.w
        ).normalize();
        let worldPos = localPos.applyQuaternion(fq).add(fp);
        let worldQuat = fq.clone().multiply(localQuat);
        tm.pose = {
          position: { x: worldPos.x, y: worldPos.y, z: worldPos.z },
          orientation: { x: worldQuat.x, y: worldQuat.y, z: worldQuat.z, w: worldQuat.w },
        };
      }

      // Transform corners (array of {x,y,z} points)
      if (m.corners && m.corners.length) {
        tm.corners = m.corners.map(function(c) {
          let p = new THREE.Vector3(c.x || 0, c.y || 0, c.z || 0);
          p.applyQuaternion(fq).add(fp);
          return { x: p.x, y: p.y, z: p.z };
        });
      } else {
        tm.corners = m.corners;
      }

      result.push(tm);
    }
    return result;
  },

  /**
   * Transform a single point {x,y,z} from frameId into rootFrameId.
   *
   * @param {Object} point - {x, y, z}
   * @param {string} frameId
   * @param {string} rootFrameId
   * @param {Object} tfTree
   * @returns {Object} {x, y, z}
   */
  transformPoint: function(point, frameId, rootFrameId, tfTree) {
    if (!point) return point;
    let ft = TFUtils.getFrameWorldTransform(frameId, rootFrameId, tfTree);
    if (!ft) return point;
    let p = new THREE.Vector3(point.x || 0, point.y || 0, point.z || 0);
    p.applyQuaternion(ft.quaternion).add(ft.position);
    return { x: p.x, y: p.y, z: p.z };
  },
};
