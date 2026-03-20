"use strict";

// TF Viewer v5.0 — Three.js + Plugin Architecture
// Orchestrates: TF frames, RobotModelPlugin, ArucoMarkerPlugin.

class TFViewer extends Viewer {
  onCreate() {
    // TF state
    this.allTransforms = {};
    this.allFrameIds = new Set();
    this.allChildFrameIds = new Set();
    this.visibleChildFrames = new Set();
    this.showLabels = true;
    this.selectedFrameId = null;
    this.axisScale = 0.3;
    this.showLinks = true;
    this._lastTree = null;

    // Three.js object pools
    this._frameAxes = {};
    this._frameLinks = {};

    // Secondary ArUco topic names we subscribed to
    this._arucoTopicNames = [];

    this._createControls();
    this._initThreeJS();

    // Labels overlay (shared by TF labels and plugins)
    this.labelsOverlay = $('<div></div>').css({
      "position": "absolute",
      "top": "0", "left": "0",
      "width": "100%", "height": "100%",
      "pointer-events": "none",
      "overflow": "hidden",
    }).appendTo(this.wrapper2);
    this.labelElements = {};

    // ── Plugins ──────────────────────────────────────────────
    this.robotPlugin = new RobotModelPlugin(this.scene, {
      modelUrl: '/models/obrik-sim2.fbx',
      targetSize: 0.35,
    });

    this.arucoPlugin = new ArucoMarkerPlugin(this.scene, this.labelsOverlay, this.camera);

    // Auto-discover ArUco topics after a short delay (topics arrive async)
    let that = this;
    this._arucoDiscoveryInterval = setInterval(() => {
      that._discoverArucoTopics();
    }, 2000);
  }

  // ── UI Controls ────────────────────────────────────────────

  _createControls() {
    this.controlsBar = $('<div></div>').css({
      "display": "flex", "flex-wrap": "wrap", "gap": "6px",
      "align-items": "center", "padding": "4px 0", "font-size": "10px",
    }).appendTo(this.card.content);

    $('<span></span>').addClass("monospace").css({"opacity": 0.6}).text("frame:").appendTo(this.controlsBar);
    this.frameIdSelect = $('<select></select>').css({
      "max-width": "120px", "font-size": "10px",
      "background": "#333", "color": "#ddd",
      "border": "1px solid #555", "border-radius": "3px", "padding": "1px 2px",
    }).appendTo(this.controlsBar);

    $('<span></span>').addClass("monospace").css({"opacity": 0.6}).text("scale:").appendTo(this.controlsBar);
    this.axisScaleInput = $('<input type="number" min="0.01" max="100" step="0.05">')
      .css({"width": "50px", "font-size": "10px", "background": "#333", "color": "#ddd",
        "border": "1px solid #555", "border-radius": "3px", "padding": "1px 2px"})
      .val(this.axisScale).appendTo(this.controlsBar);

    let linksLabel = $('<label></label>').css({"display": "flex", "gap": "3px", "align-items": "center", "cursor": "pointer"}).appendTo(this.controlsBar);
    this.showLinksCheckbox = $('<input type="checkbox" checked>').appendTo(linksLabel);
    $('<span></span>').addClass("monospace").css({"opacity": 0.6}).text("links").appendTo(linksLabel);

    let labelsLabel = $('<label></label>').css({"display": "flex", "gap": "3px", "align-items": "center", "cursor": "pointer"}).appendTo(this.controlsBar);
    this.showLabelsCheckbox = $('<input type="checkbox" checked>').appendTo(labelsLabel);
    $('<span></span>').addClass("monospace").css({"opacity": 0.6}).text("labels").appendTo(labelsLabel);

    let modelLabel = $('<label></label>').css({"display": "flex", "gap": "3px", "align-items": "center", "cursor": "pointer"}).appendTo(this.controlsBar);
    this.showModelCheckbox = $('<input type="checkbox" checked>').appendTo(modelLabel);
    $('<span></span>').addClass("monospace").css({"opacity": 0.6}).text("model").appendTo(modelLabel);

    let arucoLabel = $('<label></label>').css({"display": "flex", "gap": "3px", "align-items": "center", "cursor": "pointer"}).appendTo(this.controlsBar);
    this.showArucoCheckbox = $('<input type="checkbox" checked>').appendTo(arucoLabel);
    $('<span></span>').addClass("monospace").css({"opacity": 0.6}).text("aruco").appendTo(arucoLabel);

    // Camera mode toggle: orbit / fly
    this.cameraMode = 'orbit';
    this.cameraModeBtn = $('<button></button>').css({
      "font-size": "9px", "padding": "1px 6px",
      "background": "#444", "color": "#ccc",
      "border": "1px solid #666", "border-radius": "3px", "cursor": "pointer",
      "font-family": "'JetBrains Mono', monospace",
    }).text("orbit").appendTo(this.controlsBar);
    let that2 = this;
    this.cameraModeBtn.on("click", function() {
      that2._toggleCameraMode();
    });

    this.frameCountLabel = $('<span></span>').addClass("monospace").css({"opacity": 0.4, "margin-left": "auto"}).appendTo(this.controlsBar);

    // Collapsible child frames
    this.childToggle = $('<div></div>').css({
      "display": "flex", "align-items": "center", "gap": "4px",
      "cursor": "pointer", "padding": "2px 0", "font-size": "10px", "user-select": "none",
    }).appendTo(this.card.content);
    this.childToggleArrow = $('<span></span>').text("\u25B6").css({"opacity": 0.5, "font-size": "8px", "transition": "transform 0.2s"}).appendTo(this.childToggle);
    $('<span></span>').addClass("monospace").css({"opacity": 0.6}).text("frames").appendTo(this.childToggle);

    this.childFramesPanel = $('<div></div>').css({
      "display": "none", "max-height": "120px", "overflow-y": "auto",
      "border": "1px solid rgba(255,255,255,0.1)", "border-radius": "3px",
      "padding": "4px", "margin-bottom": "4px",
      "column-count": "2", "column-gap": "8px", "font-size": "10px",
    }).appendTo(this.card.content);
    this.childFramesList = $('<div></div>').appendTo(this.childFramesPanel);

    let childPanelVisible = false;
    this.childToggle.on("click", () => {
      childPanelVisible = !childPanelVisible;
      this.childFramesPanel.css("display", childPanelVisible ? "" : "none");
      this.childToggleArrow.css("transform", childPanelVisible ? "rotate(90deg)" : "none");
    });

    // Event handlers
    let that = this;
    this.frameIdSelect.on("change", function() {
      that.selectedFrameId = $(this).val() || null;
      that._clearFrameObjects();
      that._updateDisplay();
    });
    this.axisScaleInput.on("change input", function() {
      let v = parseFloat($(this).val());
      if (!Number.isFinite(v) || v <= 0) v = 0.3;
      that.axisScale = v;
      that._updateDisplay();
    });
    this.showLinksCheckbox.on("change", function() {
      that.showLinks = !!$(this).is(":checked");
      that._updateDisplay();
    });
    this.showLabelsCheckbox.on("change", function() {
      that.showLabels = !!$(this).is(":checked");
      that._updateLabels();
    });
    this.showModelCheckbox.on("change", function() {
      if (that.robotPlugin) that.robotPlugin.setVisible(!!$(this).is(":checked"));
    });
    this.showArucoCheckbox.on("change", function() {
      if (that.arucoPlugin) that.arucoPlugin.setVisible(!!$(this).is(":checked"));
    });
  }

  // ── Three.js Initialization ────────────────────────────────

  _initThreeJS() {
    this.wrapper = $('<div></div>').css({
      "position": "relative", "width": "100%",
    }).appendTo(this.card.content);

    this.wrapper2 = $('<div></div>').css({
      "width": "100%",
      "aspect-ratio": "1",
      "background": "#1a1a2e",
      "position": "relative",
      "overflow": "hidden",
    }).appendTo(this.wrapper);

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x1a1a2e);
    this.scene.fog = new THREE.FogExp2(0x1a1a2e, 0.035);

    this.camera = new THREE.PerspectiveCamera(50, 1, 0.01, 500);
    this.camera.position.set(4, -6, 5);
    this.camera.up.set(0, 0, 1);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setSize(500, 500);
    this.renderer.outputEncoding = THREE.sRGBEncoding;
    this.wrapper2[0].appendChild(this.renderer.domElement);
    $(this.renderer.domElement).css({
      "position": "absolute",
      "top": "0", "left": "0",
      "width": "100%", "height": "100%",
    });

    this.orbitControls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
    this.orbitControls.target.set(0, 0, 1.5);
    this.orbitControls.enableDamping = true;
    this.orbitControls.dampingFactor = 0.12;
    this.orbitControls.minDistance = 0.5;
    this.orbitControls.maxDistance = 100;
    this.orbitControls.update();

    // Lighting
    this.scene.add(new THREE.AmbientLight(0xcccccc, 0.7));
    let dirLight = new THREE.DirectionalLight(0xffffff, 0.9);
    dirLight.position.set(5, -3, 10);
    this.scene.add(dirLight);
    let fillLight = new THREE.DirectionalLight(0xffffff, 0.4);
    fillLight.position.set(-5, 5, 2);
    this.scene.add(fillLight);
    let backLight = new THREE.DirectionalLight(0x8899bb, 0.3);
    backLight.position.set(0, 0, -5);
    this.scene.add(backLight);

    this._createGrid();
    this.scene.add(new THREE.AxesHelper(0.5));

    this.framesGroup = new THREE.Group();
    this.linksGroup = new THREE.Group();
    this.scene.add(this.framesGroup);
    this.scene.add(this.linksGroup);

    // Render loop
    let that = this;
    let lastLabelTime = 0;
    let lastFrameTime = performance.now();
    let animate = () => {
      that._animFrameId = requestAnimationFrame(animate);
      let now = performance.now();
      let delta = Math.min((now - lastFrameTime) / 1000, 0.1); // seconds, capped
      lastFrameTime = now;

      if (that.cameraMode === 'fly') {
        that._updateFlyMovement(delta);
      } else {
        that.orbitControls.update();
      }
      if (that.robotPlugin) that.robotPlugin.updateSmooth(delta);
      if (that.arucoPlugin) that.arucoPlugin.updateSmooth(delta);
      that.renderer.render(that.scene, that.camera);
      if (now - lastLabelTime > 50) {
        lastLabelTime = now;
        that._updateLabels();
        if (that.arucoPlugin) that.arucoPlugin.updateLabels();
      }
    };
    animate();

    // Resize
    if (window.ResizeObserver) {
      this._resizeObserver = new ResizeObserver(() => {
        let w = that.wrapper2[0].clientWidth;
        let h = that.wrapper2[0].clientHeight;
        if (w > 0 && h > 0) {
          that.renderer.setSize(w, h);
          that.camera.aspect = w / h;
          that.camera.updateProjectionMatrix();
        }
      });
      this._resizeObserver.observe(this.wrapper2[0]);
    }
  }

  _createGrid() {
    let gridSize = 10, gridDivisions = 10;
    let verts = [], half = gridSize / 2, step = gridSize / gridDivisions;
    for (let i = 0; i <= gridDivisions; i++) {
      let pos = -half + i * step;
      verts.push(pos, -half, 0, pos, half, 0);
      verts.push(-half, pos, 0, half, pos, 0);
    }
    let geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
    this.scene.add(new THREE.LineSegments(geo,
      new THREE.LineBasicMaterial({ color: 0x334466, transparent: true, opacity: 0.3 })));
  }

  // ── Camera Mode (Orbit / Fly) ──────────────────────────────

  _toggleCameraMode() {
    if (this.cameraMode === 'orbit') {
      this._enableFlyMode();
    } else {
      this._enableOrbitMode();
    }
  }

  _enableOrbitMode() {
    this.cameraMode = 'orbit';
    this.cameraModeBtn.text('orbit').css({"background": "#444"});
    this.orbitControls.enabled = true;
    this._cleanupFlyMode();
  }

  _enableFlyMode() {
    this.cameraMode = 'fly';
    this.cameraModeBtn.text('fly').css({"background": "#335"});
    this.orbitControls.enabled = false;
    this._initFlyMode();
  }

  _initFlyMode() {
    let that = this;
    this._flyKeys = {};
    this._flySpeed = 3.0;
    this._flyDragging = false;
    this._flyLastMouse = { x: 0, y: 0 };

    // Extract current yaw/pitch from camera direction (Z-up world)
    let dir = new THREE.Vector3();
    this.camera.getWorldDirection(dir);
    this._flyYaw = Math.atan2(dir.y, dir.x);
    this._flyPitch = Math.asin(Math.max(-0.999, Math.min(0.999, dir.z)));
    this._applyFlyCamera();

    this._flyOnKeyDown = (e) => {
      that._flyKeys[e.code] = true;
      if (['KeyW','KeyA','KeyS','KeyD','KeyQ','KeyE','Space','ShiftLeft','ShiftRight'].includes(e.code)) {
        e.preventDefault();
      }
    };
    this._flyOnKeyUp = (e) => { that._flyKeys[e.code] = false; };

    this._flyOnMouseDown = (e) => {
      if (e.button === 0 || e.button === 2) {
        that._flyDragging = true;
        that._flyLastMouse = { x: e.clientX, y: e.clientY };
      }
    };
    this._flyOnMouseUp = () => { that._flyDragging = false; };
    this._flyOnMouseMove = (e) => {
      if (!that._flyDragging) return;
      let dx = e.clientX - that._flyLastMouse.x;
      let dy = e.clientY - that._flyLastMouse.y;
      that._flyLastMouse = { x: e.clientX, y: e.clientY };
      let sensitivity = 0.003;
      // Horizontal mouse → yaw (rotate around Z in Z-up world)
      that._flyYaw -= dx * sensitivity;
      // Vertical mouse → pitch (mouse down = look down, standard FPS)
      that._flyPitch -= dy * sensitivity;
      // Clamp pitch to avoid flipping
      that._flyPitch = Math.max(-Math.PI / 2 + 0.05, Math.min(Math.PI / 2 - 0.05, that._flyPitch));
      that._applyFlyCamera();
    };
    this._flyOnContextMenu = (e) => { e.preventDefault(); };

    let el = this.renderer.domElement;
    el.setAttribute('tabindex', '0');
    el.focus();
    document.addEventListener('keydown', this._flyOnKeyDown);
    document.addEventListener('keyup', this._flyOnKeyUp);
    el.addEventListener('mousedown', this._flyOnMouseDown);
    document.addEventListener('mouseup', this._flyOnMouseUp);
    document.addEventListener('mousemove', this._flyOnMouseMove);
    el.addEventListener('contextmenu', this._flyOnContextMenu);

    // Fly hint overlay
    if (!this._flyHint) {
      this._flyHint = $('<div></div>').css({
        "position": "absolute", "bottom": "6px", "left": "50%",
        "transform": "translateX(-50%)", "font-size": "8px",
        "font-family": "'JetBrains Mono', monospace",
        "color": "rgba(200,200,220,0.5)", "white-space": "nowrap",
        "pointer-events": "none",
      }).text("WASD move  QE up/down  Shift fast  drag look").appendTo(this.wrapper2);
    }
    this._flyHint.css("display", "");
  }

  /** Apply yaw/pitch to camera via lookAt (Z-up world). */
  _applyFlyCamera() {
    let dir = new THREE.Vector3(
      Math.cos(this._flyPitch) * Math.cos(this._flyYaw),
      Math.cos(this._flyPitch) * Math.sin(this._flyYaw),
      Math.sin(this._flyPitch)
    );
    let target = this.camera.position.clone().add(dir);
    this.camera.up.set(0, 0, 1);
    this.camera.lookAt(target);
  }

  _cleanupFlyMode() {
    if (this._flyOnKeyDown) {
      document.removeEventListener('keydown', this._flyOnKeyDown);
      document.removeEventListener('keyup', this._flyOnKeyUp);
      let el = this.renderer.domElement;
      el.removeEventListener('mousedown', this._flyOnMouseDown);
      document.removeEventListener('mouseup', this._flyOnMouseUp);
      document.removeEventListener('mousemove', this._flyOnMouseMove);
      el.removeEventListener('contextmenu', this._flyOnContextMenu);
    }
    this._flyKeys = {};
    this._flyOnKeyDown = null;
    if (this._flyHint) this._flyHint.css("display", "none");
  }

  _updateFlyMovement(delta) {
    if (this.cameraMode !== 'fly' || !this._flyKeys) return;
    let speed = this._flySpeed * delta;
    if (this._flyKeys['ShiftLeft'] || this._flyKeys['ShiftRight']) speed *= 3;

    // Forward direction on the horizontal XY plane (Z-up, yaw only)
    let fwdX = Math.cos(this._flyYaw);
    let fwdY = Math.sin(this._flyYaw);
    // Right = perpendicular in XY plane
    let rightX = Math.sin(this._flyYaw);
    let rightY = -Math.cos(this._flyYaw);

    let moved = false;
    if (this._flyKeys['KeyW']) { this.camera.position.x += fwdX * speed; this.camera.position.y += fwdY * speed; moved = true; }
    if (this._flyKeys['KeyS']) { this.camera.position.x -= fwdX * speed; this.camera.position.y -= fwdY * speed; moved = true; }
    if (this._flyKeys['KeyD']) { this.camera.position.x += rightX * speed; this.camera.position.y += rightY * speed; moved = true; }
    if (this._flyKeys['KeyA']) { this.camera.position.x -= rightX * speed; this.camera.position.y -= rightY * speed; moved = true; }
    if (this._flyKeys['KeyQ'] || this._flyKeys['Space']) { this.camera.position.z += speed; moved = true; }
    if (this._flyKeys['KeyE']) { this.camera.position.z -= speed; moved = true; }

    if (moved) this._applyFlyCamera();
  }

  // ── ArUco Topic Discovery ──────────────────────────────────

  _discoverArucoTopics() {
    let topics = Viewer._topics;
    if (!topics) return;

    let arucoTypes = [
      'aruco_det_loc/msg/MarkerArray',
      'visualization_msgs/msg/MarkerArray',
      'visualization_msgs/MarkerArray',
    ];

    for (let topicName in topics) {
      let topicType = topics[topicName];
      if (arucoTypes.includes(topicType) && !this._arucoTopicNames.includes(topicName)) {
        this._arucoTopicNames.push(topicName);
        let that = this;
        Viewer.subscribeSecondary(topicName, (msg) => {
          that._onArucoData(msg);
        });
        console.log('[TFViewer] Auto-subscribed to ArUco topic:', topicName);
      }
    }

    // Stop polling once we found topics (or after topics are available)
    if (Object.keys(topics).length > 0) {
      clearInterval(this._arucoDiscoveryInterval);
      this._arucoDiscoveryInterval = null;
    }
  }

  _onArucoData(msg) {
    if (!this.arucoPlugin) return;
    let markers = msg.markers || [];
    if (markers.length === 0) return;

    let rootId = this.selectedFrameId || '';

    // aruco_det_loc/msg/MarkerArray: common header.frame_id
    // visualization_msgs/msg/MarkerArray: per-marker header.frame_id (no common header)
    let hasPerMarkerHeader = !!(markers[0] && markers[0].header && typeof markers[0].header.frame_id === "string");

    if (hasPerMarkerHeader) {
      // Convert visualization_msgs/Marker into our marker format {id,size,pose,corners}
      let converted = [];
      for (let i = 0; i < markers.length; i++) {
        let m = markers[i];
        if (m == null) continue;
        if (m.id == null) continue;

        let frameId = m.header?.frame_id || "";
        if (!frameId) continue;

        // visualization_msgs/Marker: pose is {position, orientation}
        let pose = m.pose ? { position: m.pose.position, orientation: m.pose.orientation } : null;
        let size = (m.scale && (m.scale.x || m.scale.y)) ? Math.max(m.scale.x || 0, m.scale.y || 0) : undefined;

        converted.push({ id: m.id, size, pose, corners: null, _frame_id: frameId });
      }

      // Transform each marker from its own frame into root
      let transformed = [];
      for (let i = 0; i < converted.length; i++) {
        let cm = converted[i];
        let one = TFUtils.transformMarkers([cm], cm._frame_id, rootId, this._lastTree);
        if (one && one.length) transformed.push(one[0]);
      }
      this.arucoPlugin.updateMarkers(transformed);
      return;
    }

    let frameId = msg.header?.frame_id || "";
    if (!frameId) return;

    // Transform markers from source frame into root TF frame
    let transformed = TFUtils.transformMarkers(markers, frameId, rootId, this._lastTree);
    this.arucoPlugin.updateMarkers(transformed);
  }

  // ── TF Data Management ────────────────────────────────────

  _storeTransform(transform) {
    let parent = transform?.header?.frame_id;
    let child = transform?.child_frame_id;
    if (!parent || !child) return false;
    this.allTransforms[parent + "->" + child] = transform;
    let wasNew = !this.allFrameIds.has(parent) || !this.allChildFrameIds.has(child);
    this.allFrameIds.add(parent);
    this.allChildFrameIds.add(child);
    return wasNew;
  }

  _updateFrameIdOptions() {
    let frameIds = Array.from(this.allFrameIds).sort();
    let currentValue = this.frameIdSelect.val();
    this.frameIdSelect.empty();

    if (frameIds.length === 0) {
      $('<option></option>').text("(none)").appendTo(this.frameIdSelect);
      this.selectedFrameId = null;
      return;
    }
    frameIds.forEach((fid) => {
      $('<option></option>').attr("value", fid).text(fid).appendTo(this.frameIdSelect);
    });
    if (currentValue && frameIds.includes(currentValue)) {
      this.frameIdSelect.val(currentValue);
      this.selectedFrameId = currentValue;
    } else if (!this.selectedFrameId || !frameIds.includes(this.selectedFrameId)) {
      this.selectedFrameId = frameIds.includes("map") ? "map" : frameIds[0];
      this.frameIdSelect.val(this.selectedFrameId);
    }
  }

  _updateChildFramesList() {
    let childFrames = Array.from(this.allChildFrameIds).sort();
    this.childFramesList.empty();
    if (childFrames.length === 0) return;
    if (this.visibleChildFrames.size === 0) {
      childFrames.forEach((cfid) => this.visibleChildFrames.add(cfid));
    }
    let that = this;
    childFrames.forEach((cfid) => {
      let row = $('<label></label>').css({
        "display": "flex", "gap": "3px", "align-items": "center",
        "cursor": "pointer", "break-inside": "avoid", "line-height": "1.6",
      }).appendTo(this.childFramesList);
      let checkbox = $('<input type="checkbox">')
        .prop("checked", this.visibleChildFrames.has(cfid)).appendTo(row);
      checkbox.on("change", function() {
        if ($(this).is(":checked")) that.visibleChildFrames.add(cfid);
        else that.visibleChildFrames.delete(cfid);
        that._updateDisplay();
      });
      $('<span></span>').addClass("monospace").css({"opacity": 0.8}).text(cfid).appendTo(row);
    });
    this.frameCountLabel.text(childFrames.length + " frames");
  }

  // ── Transform Tree ─────────────────────────────────────────

  _buildTransformTree(rootFrameId) {
    let tree = {};
    let visited = new Set();
    let buildTree = (frameId, parentPos, parentQuat) => {
      if (visited.has(frameId)) return;
      visited.add(frameId);
      for (let key in this.allTransforms) {
        let tf = this.allTransforms[key];
        if (tf?.header?.frame_id !== frameId) continue;
        let childId = tf?.child_frame_id;
        if (!childId) continue;
        let tr = tf?.transform?.translation;
        let rot = tf?.transform?.rotation;
        if (!tr || !rot) continue;
        let localPos = new THREE.Vector3(tr.x || 0, tr.y || 0, tr.z || 0);
        let localQuat = new THREE.Quaternion(
          rot.x || 0, rot.y || 0, rot.z || 0, rot.w == null ? 1 : rot.w
        ).normalize();
        let childPos = localPos.clone().applyQuaternion(parentQuat).add(parentPos);
        let childQuat = parentQuat.clone().multiply(localQuat);
        tree[childId] = {
          transform: tf, position: childPos, quaternion: childQuat,
          parentFrameId: frameId, parentPosition: parentPos.clone(),
        };
        buildTree(childId, childPos, childQuat);
      }
    };
    buildTree(rootFrameId, new THREE.Vector3(), new THREE.Quaternion());
    return tree;
  }

  // ── 3D Scene Update ────────────────────────────────────────

  _clearFrameObjects() {
    for (let id in this._frameAxes) {
      this.framesGroup.remove(this._frameAxes[id]);
      this._frameAxes[id].dispose();
    }
    this._frameAxes = {};
    for (let id in this._frameLinks) {
      this.linksGroup.remove(this._frameLinks[id]);
      this._frameLinks[id].geometry.dispose();
      this._frameLinks[id].material.dispose();
    }
    this._frameLinks = {};
  }

  _updateDisplay() {
    if (!this.selectedFrameId) return;
    let scale = this.axisScale;
    if (!Number.isFinite(scale) || scale <= 0) scale = 0.3;

    let tree = this._buildTransformTree(this.selectedFrameId);
    this._lastTree = tree;
    let activeChildIds = new Set();

    for (let childId in tree) {
      if (!this.visibleChildFrames.has(childId)) continue;
      activeChildIds.add(childId);
      let fd = tree[childId];

      // Frame axes
      if (!this._frameAxes[childId]) {
        let axes = new THREE.AxesHelper(1.0);
        this.framesGroup.add(axes);
        this._frameAxes[childId] = axes;
      }
      let axes = this._frameAxes[childId];
      axes.position.copy(fd.position);
      axes.quaternion.copy(fd.quaternion);
      axes.scale.setScalar(scale);
      axes.visible = true;

      // Link lines
      if (this.showLinks && fd.parentPosition) {
        if (!this._frameLinks[childId]) {
          let mat = new THREE.LineBasicMaterial({ color: 0x667788, transparent: true, opacity: 0.35 });
          let geo = new THREE.BufferGeometry();
          geo.setAttribute('position', new THREE.Float32BufferAttribute([0,0,0, 0,0,0], 3));
          this.linksGroup.add(new THREE.Line(geo, mat));
          this._frameLinks[childId] = this.linksGroup.children[this.linksGroup.children.length - 1];
        }
        let posAttr = this._frameLinks[childId].geometry.attributes.position;
        posAttr.setXYZ(0, fd.parentPosition.x, fd.parentPosition.y, fd.parentPosition.z);
        posAttr.setXYZ(1, fd.position.x, fd.position.y, fd.position.z);
        posAttr.needsUpdate = true;
        this._frameLinks[childId].visible = true;
      } else if (this._frameLinks[childId]) {
        this._frameLinks[childId].visible = false;
      }

      // Robot model follows base_link
      if (childId === 'base_link' && this.robotPlugin) {
        this.robotPlugin.setTransform(fd.position, fd.quaternion);
      }
    }

    // Robot model always follows base_link, even if frame is hidden
    if (!activeChildIds.has('base_link') && tree['base_link'] && this.robotPlugin) {
      this.robotPlugin.setTransform(tree['base_link'].position, tree['base_link'].quaternion);
    }

    // Hide inactive
    for (let id in this._frameAxes) {
      if (!activeChildIds.has(id)) this._frameAxes[id].visible = false;
    }
    for (let id in this._frameLinks) {
      if (!activeChildIds.has(id)) this._frameLinks[id].visible = false;
    }

    this._updateLabels();
  }

  // ── Labels (HTML overlay for TF frames) ────────────────────

  _project3DTo2D(pos) {
    let v = pos instanceof THREE.Vector3 ? pos.clone() : new THREE.Vector3(pos[0], pos[1], pos[2]);
    v.project(this.camera);
    if (Math.abs(v.x) > 2 || Math.abs(v.y) > 2 || v.z > 1) return null;
    return { x: (v.x * 0.5 + 0.5) * 100, y: (-v.y * 0.5 + 0.5) * 100 };
  }

  _updateLabels() {
    if (!this.labelsOverlay) return;
    if (!this.showLabels || !this._lastTree) {
      // Only hide TF labels, not aruco labels
      for (let name in this.labelElements) {
        this.labelElements[name].css("display", "none");
      }
      return;
    }

    let usedLabels = new Set();
    let rootName = this.selectedFrameId;
    if (rootName) {
      usedLabels.add(rootName);
      this._setLabel(rootName, this._project3DTo2D(new THREE.Vector3(0, 0, 0)), "#fff");
    }
    for (let childId in this._lastTree) {
      if (!this.visibleChildFrames.has(childId)) continue;
      usedLabels.add(childId);
      this._setLabel(childId, this._project3DTo2D(this._lastTree[childId].position), "#ccc");
    }
    for (let name in this.labelElements) {
      if (!usedLabels.has(name)) {
        this.labelElements[name].remove();
        delete this.labelElements[name];
      }
    }
  }

  _setLabel(name, screenPos, color) {
    if (!screenPos || screenPos.x < -10 || screenPos.x > 110 || screenPos.y < -10 || screenPos.y > 110) {
      if (this.labelElements[name]) this.labelElements[name].css("display", "none");
      return;
    }
    if (!this.labelElements[name]) {
      this.labelElements[name] = $('<div></div>').css({
        "position": "absolute",
        "font-size": "9px",
        "font-family": "'JetBrains Mono', monospace",
        "color": color,
        "text-shadow": "0 0 3px #000, 0 0 6px #000",
        "white-space": "nowrap",
        "pointer-events": "none",
        "transform": "translate(-50%, -100%)",
        "padding-bottom": "2px",
      }).appendTo(this.labelsOverlay);
    }
    this.labelElements[name].text(name).css({
      "display": "", "left": screenPos.x + "%", "top": screenPos.y + "%",
    });
  }

  // ── Data Handler ───────────────────────────────────────────

  onData(msg) {
    this._lastMsg = msg;
    this.card.title.text(msg._topic_name);

    let transforms = msg.transforms || [];
    if (!Array.isArray(transforms) || transforms.length === 0) return;

    let hasNewFrames = false;
    let prevCount = this.allChildFrameIds.size;
    for (let i = 0; i < transforms.length; i++) {
      if (this._storeTransform(transforms[i])) hasNewFrames = true;
    }
    if (hasNewFrames) {
      this._updateFrameIdOptions();
      if (this.allChildFrameIds.size > prevCount) {
        this.allChildFrameIds.forEach((cfid) => this.visibleChildFrames.add(cfid));
      }
      this._updateChildFramesList();
    }
    this._updateDisplay();
  }

  // ── Cleanup ────────────────────────────────────────────────

  destroy() {
    // Unsubscribe secondary topics
    for (let i = 0; i < this._arucoTopicNames.length; i++) {
      Viewer.unsubscribeSecondary(this._arucoTopicNames[i]);
    }
    if (this._arucoDiscoveryInterval) clearInterval(this._arucoDiscoveryInterval);

    // Destroy plugins
    if (this.robotPlugin) this.robotPlugin.destroy();
    if (this.arucoPlugin) this.arucoPlugin.destroy();

    // Three.js cleanup
    this._cleanupFlyMode();
    if (this._animFrameId) cancelAnimationFrame(this._animFrameId);
    if (this._resizeObserver) this._resizeObserver.disconnect();
    this._clearFrameObjects();
    if (this.orbitControls) this.orbitControls.dispose();
    if (this.renderer) this.renderer.dispose();
  }
}

TFViewer.friendlyName = "TF (3D)";
TFViewer.supportedTypes = [
  "tf2_msgs/msg/TFMessage",
];
TFViewer.maxUpdateRate = 30.0;

Viewer.registerViewer(TFViewer);
