"use strict";

// GenericViewer v2.1
// Tree-view display for any ROS message type.
// Stable DOM — only values update, structure rebuilds only when keys change.

class GenericViewer extends Viewer {
  onCreate() {
    this.viewerNode = $('<div></div>')
      .css({'font-size': '11px', 'font-family': "'JetBrains Mono', monospace"})
      .appendTo(this.card.content);

    this.treeContainer = $('<div></div>')
      .css({'padding': '6px', 'line-height': '1.6'})
      .appendTo(this.viewerNode);

    // Track expanded paths
    this.expandedPaths = new Set();
    // Cache value spans by path for in-place updates
    this.valueSpans = {};
    // Last structure signature to detect when keys change
    this.lastStructureSig = "";
    // Last data for re-rendering on expand/collapse
    this.lastData = null;

    super.onCreate();
  }

  onData(data) {
    this.card.title.text(data._topic_name);
    this.lastData = data;

    // Check if structure changed (new/removed keys)
    let sig = this._structureSig(data);
    if(sig !== this.lastStructureSig) {
      // Full rebuild
      this.lastStructureSig = sig;
      this.treeContainer.empty();
      this.valueSpans = {};
      this._buildTree(data, this.treeContainer, "", true);
    } else {
      // Fast update — only change value text
      this._updateValues(data, "");
    }
  }

  _structureSig(obj) {
    // Recursive signature of key structure (not values) — detects new/removed keys at any depth
    if(obj === null || obj === undefined || typeof obj !== 'object') return "";
    let keys = Object.keys(obj).filter(k => k[0] !== '_').sort();
    let parts = [];
    for(let k of keys) {
      let v = obj[k];
      if(v !== null && typeof v === 'object' && !Array.isArray(v)) {
        parts.push(k + ":{" + this._structureSig(v) + "}");
      } else if(Array.isArray(v)) {
        parts.push(k + ":[" + v.length + "]");
      } else {
        parts.push(k);
      }
    }
    return parts.join(",");
  }

  _updateValues(obj, path) {
    if(obj === null || obj === undefined || typeof obj !== 'object') return;

    let keys = Object.keys(obj).filter(k => k[0] !== '_');
    for(let key of keys) {
      let childPath = path ? path + "." + key : key;
      let value = obj[key];

      if(value !== null && typeof value === 'object' && !Array.isArray(value)) {
        // Update count badge
        let countSpan = this.valueSpans[childPath + ".__count"];
        if(countSpan) {
          let n = Object.keys(value).filter(k => k[0] !== '_').length;
          countSpan.text("{" + n + "}");
        }
        // Recurse if expanded
        if(this.expandedPaths.has(childPath)) {
          this._updateValues(value, childPath);
        }
      } else if(Array.isArray(value)) {
        // Update count & preview
        let countSpan = this.valueSpans[childPath + ".__count"];
        if(countSpan) countSpan.text("[" + value.length + "]");

        let previewSpan = this.valueSpans[childPath + ".__preview"];
        if(previewSpan && value.length <= 6 && value.every(v => typeof v === 'number')) {
          previewSpan.text(value.map(v => this._fmtNum(v)).join(", "));
        }

        // Update array items if expanded
        if(this.expandedPaths.has(childPath)) {
          this._updateArrayValues(value, childPath);
        }
      } else {
        // Update primitive value span
        let span = this.valueSpans[childPath];
        if(span) this._setValueSpan(span, value);
      }
    }
  }

  _updateArrayValues(arr, path) {
    for(let i = 0; i < Math.min(arr.length, 100); i++) {
      let childPath = path + "[" + i + "]";
      let item = arr[i];
      if(item !== null && typeof item === 'object' && !Array.isArray(item)) {
        if(this.expandedPaths.has(childPath)) {
          this._updateValues(item, childPath);
        }
      } else {
        let span = this.valueSpans[childPath];
        if(span) this._setValueSpan(span, item);
      }
    }
  }

  _buildTree(obj, container, path, isRoot) {
    if(obj === null || obj === undefined) return;
    let keys = Object.keys(obj).filter(k => k[0] !== '_');

    for(let key of keys) {
      let value = obj[key];
      let childPath = path ? path + "." + key : key;

      let row = $('<div></div>').css({
        "padding-left": isRoot ? "0" : "14px",
        "border-left": isRoot ? "none" : "1px solid rgba(255,255,255,0.06)",
      }).appendTo(container);

      if(value !== null && typeof value === 'object' && !Array.isArray(value)) {
        this._buildObjectNode(row, key, value, childPath);
      } else if(Array.isArray(value)) {
        this._buildArrayNode(row, key, value, childPath);
      } else {
        this._buildValueNode(row, key, value, childPath);
      }
    }
  }

  _buildObjectNode(row, key, value, path) {
    let isExpanded = this.expandedPaths.has(path);

    let header = $('<div></div>').css({
      "cursor": "pointer", "padding": "1px 0",
      "display": "flex", "align-items": "center", "gap": "4px",
    }).appendTo(row);

    let arrow = $('<span></span>').text(isExpanded ? "▼" : "▶").css({
      "font-size": "8px", "opacity": 0.4, "width": "10px", "flex-shrink": 0,
    }).appendTo(header);

    $('<span></span>').text(key).css({"color": "#8be9fd", "font-weight": "bold"}).appendTo(header);

    let fieldCount = Object.keys(value).filter(k => k[0] !== '_').length;
    let countSpan = $('<span></span>').text("{" + fieldCount + "}").css({
      "color": "rgba(255,255,255,0.25)", "font-size": "9px", "margin-left": "4px",
    }).appendTo(header);
    this.valueSpans[path + ".__count"] = countSpan;

    let childContainer = $('<div></div>').css({
      "display": isExpanded ? "" : "none",
    }).appendTo(row);

    if(isExpanded) {
      this._buildTree(value, childContainer, path, false);
    }

    let that = this;
    header.on("click", function(e) {
      e.stopPropagation();
      let nowExpanded = that.expandedPaths.has(path);
      if(nowExpanded) {
        that.expandedPaths.delete(path);
        arrow.text("▶");
        childContainer.css("display", "none").empty();
        // Clean cached spans under this path
        that._cleanSpansUnder(path);
      } else {
        that.expandedPaths.add(path);
        arrow.text("▼");
        childContainer.css("display", "").empty();
        // Rebuild children using latest data
        let currentValue = that._getValueByPath(that.lastData, path);
        if(currentValue) that._buildTree(currentValue, childContainer, path, false);
      }
    });
  }

  _buildArrayNode(row, key, value, path) {
    let isExpanded = this.expandedPaths.has(path);

    let header = $('<div></div>').css({
      "cursor": "pointer", "padding": "1px 0",
      "display": "flex", "align-items": "center", "gap": "4px",
    }).appendTo(row);

    let arrow = $('<span></span>').text(isExpanded ? "▼" : "▶").css({
      "font-size": "8px", "opacity": 0.4, "width": "10px", "flex-shrink": 0,
    }).appendTo(header);

    $('<span></span>').text(key).css({"color": "#8be9fd", "font-weight": "bold"}).appendTo(header);

    let countSpan = $('<span></span>').text("[" + value.length + "]").css({
      "color": "rgba(255,255,255,0.25)", "font-size": "9px", "margin-left": "4px",
    }).appendTo(header);
    this.valueSpans[path + ".__count"] = countSpan;

    // Compact preview for small numeric arrays
    let previewSpan = null;
    if(value.length <= 6 && value.every(v => typeof v === 'number')) {
      previewSpan = $('<span></span>').text(value.map(v => this._fmtNum(v)).join(", ")).css({
        "color": "#bd93f9", "font-size": "10px", "margin-left": "6px", "opacity": 0.7,
      }).appendTo(header);
      this.valueSpans[path + ".__preview"] = previewSpan;
    }

    let childContainer = $('<div></div>').css({
      "display": isExpanded ? "" : "none",
    }).appendTo(row);

    if(isExpanded) {
      this._buildArrayItems(value, childContainer, path);
    }

    let that = this;
    header.on("click", function(e) {
      e.stopPropagation();
      let nowExpanded = that.expandedPaths.has(path);
      if(nowExpanded) {
        that.expandedPaths.delete(path);
        arrow.text("▶");
        childContainer.css("display", "none").empty();
        that._cleanSpansUnder(path);
      } else {
        that.expandedPaths.add(path);
        arrow.text("▼");
        childContainer.css("display", "").empty();
        let currentValue = that._getValueByPath(that.lastData, path);
        if(Array.isArray(currentValue)) that._buildArrayItems(currentValue, childContainer, path);
      }
    });
  }

  _buildArrayItems(arr, container, path) {
    // Large numeric arrays — compact
    if(arr.length > 20 && arr.every(v => typeof v === 'number')) {
      let text = "[" + arr.map(v => this._fmtNum(v)).join(", ") + "]";
      $('<div></div>').css({
        "padding-left": "14px", "color": "#bd93f9",
        "word-break": "break-all", "font-size": "10px",
        "max-height": "80px", "overflow-y": "auto",
      }).text(text).appendTo(container);
      return;
    }

    let limit = Math.min(arr.length, 100);
    for(let i = 0; i < limit; i++) {
      let childPath = path + "[" + i + "]";
      let item = arr[i];
      let itemRow = $('<div></div>').css({
        "padding-left": "14px",
        "border-left": "1px solid rgba(255,255,255,0.06)",
      }).appendTo(container);

      if(item !== null && typeof item === 'object' && !Array.isArray(item)) {
        this._buildObjectNode(itemRow, "[" + i + "]", item, childPath);
      } else {
        let line = $('<div></div>').css({
          "display": "flex", "align-items": "baseline", "gap": "6px",
        }).appendTo(itemRow);
        $('<span></span>').text("[" + i + "]").css({"color": "#6272a4", "min-width": "30px"}).appendTo(line);
        let valSpan = $('<span></span>').appendTo(line);
        this._setValueSpan(valSpan, item);
        this.valueSpans[childPath] = valSpan;
      }
    }

    if(arr.length > 100) {
      $('<div></div>').css({
        "padding-left": "14px", "color": "rgba(255,255,255,0.3)", "font-style": "italic",
      }).text("... +" + (arr.length - 100) + " more").appendTo(container);
    }
  }

  _buildValueNode(row, key, value, path) {
    let line = $('<div></div>').css({
      "padding": "1px 0", "display": "flex", "align-items": "baseline", "gap": "6px",
      "padding-left": "10px",
    }).appendTo(row);

    $('<span></span>').text(key).css({"color": "#8be9fd"}).appendTo(line);

    let valSpan = $('<span></span>').appendTo(line);
    this._setValueSpan(valSpan, value);
    this.valueSpans[path] = valSpan;
  }

  _setValueSpan(span, value) {
    if(value === null || value === undefined) {
      span.text("null").css({"color": "#6272a4", "font-style": "italic", "font-weight": ""});
    } else if(typeof value === 'boolean') {
      span.text(value ? "true" : "false").css({
        "color": value ? "#50fa7b" : "#ff5555", "font-weight": "bold", "font-style": "",
      });
    } else if(typeof value === 'number') {
      span.text(this._fmtNum(value)).css({"color": "#bd93f9", "font-weight": "", "font-style": ""});
    } else if(typeof value === 'string') {
      span.text('"' + value + '"').css({"color": "#f1fa8c", "font-weight": "", "font-style": ""});
    } else {
      span.text(JSON.stringify(value)).css({"color": "#ddd", "font-weight": "", "font-style": ""});
    }
  }

  _fmtNum(n) {
    if(!Number.isFinite(n)) return String(n);
    if(Number.isInteger(n)) return String(n);
    let s = n.toFixed(4).replace(/0+$/, '');
    if(s.endsWith('.')) s += '0';
    return s;
  }

  _getValueByPath(obj, path) {
    // Navigate obj by dotted path with [N] array indices
    if(!obj || !path) return obj;
    let parts = path.replace(/\[(\d+)\]/g, '.$1').split('.');
    let current = obj;
    for(let p of parts) {
      if(current === null || current === undefined) return undefined;
      current = current[p];
    }
    return current;
  }

  _cleanSpansUnder(prefix) {
    for(let key in this.valueSpans) {
      if(key.startsWith(prefix + ".") || key.startsWith(prefix + "[")) {
        delete this.valueSpans[key];
      }
    }
  }
}

GenericViewer.friendlyName = "Raw data";

GenericViewer.supportedTypes = [
  "*",
];

Viewer.registerViewer(GenericViewer);
