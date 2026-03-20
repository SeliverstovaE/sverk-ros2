"use strict";

importJsOnce("js/viewers/meta/Viewer.js");
importJsOnce("js/viewers/meta/Space2DViewer.js");
importJsOnce("js/viewers/meta/Space3DViewer.js");

// TFViewer plugins (must load before TFViewer)
importJsOnce("js/viewers/plugins/SmoothTransform.js");
importJsOnce("js/viewers/plugins/TFUtils.js");
importJsOnce("js/viewers/plugins/RobotModelPlugin.js");
importJsOnce("js/viewers/plugins/ArucoMarkerPlugin.js");

importJsOnce("js/viewers/TFViewer.js");
importJsOnce("js/viewers/ImageViewer.js");
importJsOnce("js/viewers/LogViewer.js");
importJsOnce("js/viewers/ProcessListViewer.js");
importJsOnce("js/viewers/MapViewer.js");
importJsOnce("js/viewers/LaserScanViewer.js");
importJsOnce("js/viewers/GeometryViewer.js");
importJsOnce("js/viewers/PolygonViewer.js");
importJsOnce("js/viewers/DiagnosticViewer.js");
importJsOnce("js/viewers/TimeSeriesPlotViewer.js");
importJsOnce("js/viewers/PointCloud2Viewer.js");
importJsOnce("js/viewers/ImuViewer.js");
importJsOnce("js/viewers/JointStateViewer.js");

// GenericViewer must be last
importJsOnce("js/viewers/GenericViewer.js");

importJsOnce("js/transports/WebSocketV1Transport.js");

var snackbarContainer = document.querySelector('#demo-toast-example');

let subscriptions = {};

if(window.localStorage && window.localStorage.subscriptions) {
  if(window.location.search && window.location.search.indexOf("reset") !== -1) {
    subscriptions = {};
    updateStoredSubscriptions();
    window.location.href = "?";
  } else {
    try {
      subscriptions = JSON.parse(window.localStorage.subscriptions);
    } catch(e) {
      console.log(e);
      subscriptions = {};
    }
  }
}

let $grid = null;
$(() => {
  $grid = $('.grid').masonry({
    itemSelector: '.card',
    gutter: 10,
    percentPosition: true,
  });
  $grid.masonry("layout");
});

setInterval(() => {
  if(currentTransport && !currentTransport.isConnected()) {
    console.log("attempting to reconnect ...");
    currentTransport.connect();
  }
}, 5000);

function updateStoredSubscriptions() {
  if(window.localStorage) {
    let storedSubscriptions = {};
    for(let topicName in subscriptions) {
      storedSubscriptions[topicName] = {
        topicType: subscriptions[topicName].topicType,
      };
    }
    window.localStorage['subscriptions'] = JSON.stringify(storedSubscriptions);
  }
}

function newCard() {
  // creates a new card, adds it to the grid, and returns it.
  let card = $("<div></div>").addClass('card')
    .appendTo($('.grid'));
  return card;
}

let onOpen = function() {
  const urlParams = new URLSearchParams(window.location.search);

  for( let [key, value] of urlParams ){
    key = key.replace(/\\/g, '/');
    value = value.replace(/\\/g, '/');

    console.log("Auto subscribing to " + key + " of type " + value);
      
    const subscriptions = JSON.parse(window.localStorage.getItem('subscriptions') || '{}');
    if (!(key in subscriptions)) {
      initSubscribe({topicName: key, topicType: value});
    }
  }          
  
  for(let topic_name in subscriptions) {
    console.log("Re-subscribing to " + topic_name);
    initSubscribe({topicName: topic_name, topicType: subscriptions[topic_name].topicType});
  }


}

let onSystem = function(system) {
  if(system.hostname) {
    console.log("hostname: " + system.hostname);
    $('.mdl-layout-title').text("ROSboard: " + system.hostname);
  }

  if(system.version) {
    console.log("server version: " + system.version);
    versionCheck(system.version);
  }
}

let onMsg = function(msg) {
  if(subscriptions[msg._topic_name] && subscriptions[msg._topic_name].viewer) {
    subscriptions[msg._topic_name].viewer.update(msg);
  } else if(Viewer._secondaryHandlers[msg._topic_name]) {
    Viewer._secondaryHandlers[msg._topic_name](msg);
  } else if(!subscriptions[msg._topic_name]) {
    console.log("Received unsolicited message", msg);
  } else {
    console.log("Received msg but no viewer", msg);
  }
}

let currentTopics = {};
let currentTopicsStr = "";

let onTopics = function(topics) {
  
  // check if topics has actually changed, if not, don't do anything
  // lazy shortcut to deep compares, might possibly even be faster than
  // implementing a deep compare due to
  // native optimization of JSON.stringify
  let newTopicsStr = JSON.stringify(topics);
  if(newTopicsStr === currentTopicsStr) return;
  currentTopics = topics;
  currentTopicsStr = newTopicsStr;
  
  let topicTree = treeifyPaths(Object.keys(topics));
  
  $("#topics-nav-ros").empty();
  $("#topics-nav-system").empty();
  
  addTopicTreeToNav(topicTree[0], $('#topics-nav-ros'));

  $('<a></a>')
  .addClass("mdl-navigation__link")
  .click(() => { initSubscribe({topicName: "_dmesg", topicType: "rcl_interfaces/msg/Log"}); })
  .text("dmesg")
  .appendTo($("#topics-nav-system"));

  $('<a></a>')
  .addClass("mdl-navigation__link")
  .click(() => { initSubscribe({topicName: "_top", topicType: "rosboard_msgs/msg/ProcessList"}); })
  .text("Processes")
  .appendTo($("#topics-nav-system"));

  $('<a></a>')
  .addClass("mdl-navigation__link")
  .click(() => { initSubscribe({topicName: "_system_stats", topicType: "rosboard_msgs/msg/SystemStats"}); })
  .text("System stats")
  .appendTo($("#topics-nav-system"));
}

function addTopicTreeToNav(topicTree, el, level = 0, path = "") {
  topicTree.children.sort((a, b) => {
    if(a.name>b.name) return 1;
    if(a.name<b.name) return -1;
    return 0;
  });
  topicTree.children.forEach((subTree, i) => {
    let subEl = $('<div></div>')
    .css(level < 1 ? {} : {
      "padding-left": "0pt",
      "margin-left": "12pt",
      "border-left": "1px dashed rgba(128,128,128,0.3)",
    })
    .appendTo(el);
    let fullTopicName = path + "/" + subTree.name;
    let topicType = currentTopics[fullTopicName];
    let hasChildren = subTree.children && subTree.children.length > 0;

    if(topicType) {
      // Leaf topic — clickable to subscribe
      $('<a></a>')
        .addClass("mdl-navigation__link topic-leaf")
        .css({"padding-left": "12pt", "margin-left": 0})
        .click(() => { initSubscribe({topicName: fullTopicName, topicType: topicType}); })
        .text(subTree.name)
        .appendTo(subEl);
    } else if(hasChildren) {
      // Group node — collapsible
      let childContainer = null;
      let expanded = level < 1; // top-level expanded by default, deeper collapsed
      let arrow = $('<span></span>').css({
        "display": "inline-block", "width": "12px", "font-size": "8px",
        "transition": "transform 0.15s", "text-align": "center",
        "transform": expanded ? "rotate(90deg)" : "none",
      }).text("\u25B6");

      $('<a></a>')
        .addClass("mdl-navigation__link topic-group")
        .css({
          "padding-left": "8pt", "margin-left": 0,
          "cursor": "pointer", "user-select": "none", "opacity": 0.7,
        })
        .append(arrow)
        .append($('<span></span>').text(" " + subTree.name).css({"font-weight": "600"}))
        .click(function() {
          expanded = !expanded;
          arrow.css("transform", expanded ? "rotate(90deg)" : "none");
          childContainer.css("display", expanded ? "" : "none");
        })
        .appendTo(subEl);

      childContainer = $('<div></div>').css({
        "display": expanded ? "" : "none",
      }).appendTo(subEl);

      addTopicTreeToNav(subTree, childContainer, level + 1, fullTopicName);
      return; // children already added into childContainer
    } else {
      // Namespace without children or topic — show as disabled
      $('<a></a>')
        .addClass("mdl-navigation__link")
        .attr("disabled", "disabled")
        .css({"padding-left": "12pt", "margin-left": 0, opacity: 0.35})
        .text(subTree.name)
        .appendTo(subEl);
    }

    if(hasChildren) {
      addTopicTreeToNav(subTree, subEl, level + 1, fullTopicName);
    }
  });
}

function initSubscribe({topicName, topicType}) {
  console.log( "Subscribing to " + topicName + " of type " + topicType);
  // creates a subscriber for topicName
  // and also initializes a viewer (if it doesn't already exist)
  // in advance of arrival of the first data
  // this way the user gets a snappy UI response because the viewer appears immediately
  if(!subscriptions[topicName]) {
    subscriptions[topicName] = {
      topicType: topicType,
    }
  }  
  currentTransport.subscribe({topicName: topicName});
  if(!subscriptions[topicName].viewer) {
    let card = newCard();
    let viewer = Viewer.getDefaultViewerForType(topicType);
    try {
      subscriptions[topicName].viewer = new viewer(card, topicName, topicType);
    } catch(e) {
      console.log(e);
      card.remove();
    }
    $grid.masonry("appended", card);
    $grid.masonry("layout");
  }
  updateStoredSubscriptions();
}

let currentTransport = null;

function initDefaultTransport() {
  currentTransport = new WebSocketV1Transport({
    path: "/rosboard/v1",
    onOpen: onOpen,
    onMsg: onMsg,
    onTopics: function(topics) {
      onTopics(topics);
      Viewer._topics = currentTopics; // expose to plugins
    },
    onSystem: onSystem,
  });
  Viewer._transport = currentTransport;
  currentTransport.connect();
}

function treeifyPaths(paths) {
  // turn a bunch of ros topics into a tree
  let result = [];
  let level = {result};

  paths.forEach(path => {
    path.split('/').reduce((r, name, i, a) => {
      if(!r[name]) {
        r[name] = {result: []};
        r.result.push({name, children: r[name].result})
      }
      
      return r[name];
    }, level)
  });
  return result;
}

let lastBotherTime = 0.0;
function versionCheck(currentVersionText) {
  $.get("https://raw.githubusercontent.com/dheera/rosboard/release/setup.py").done((data) => {
    let matches = data.match(/version='(.*)'/);
    if(matches.length < 2) return;
    let latestVersion = matches[1].split(".").map(num => parseInt(num, 10));
    let currentVersion = currentVersionText.split(".").map(num => parseInt(num, 10));
    let latestVersionInt = latestVersion[0] * 1000000 + latestVersion[1] * 1000 + latestVersion[2];
    let currentVersionInt = currentVersion[0] * 1000000 + currentVersion[1] * 1000 + currentVersion[2];
    if(currentVersion < latestVersion && Date.now() - lastBotherTime > 1800000) {
      lastBotherTime = Date.now();
      snackbarContainer.MaterialSnackbar.showSnackbar({
        message: "New version of ROSboard available (" + currentVersionText + " -> " + matches[1] + ").",
        actionText: "Check it out",
        actionHandler: ()=> {window.location.href="https://github.com/dheera/rosboard/"},
      });
    }
  });
}

$(() => {
  if(window.location.href.indexOf("rosboard.com") === -1) {
    initDefaultTransport();
  }

  // Collapsible section headers (System, ROS topics)
  $(".topics-nav-collapsible").each(function() {
    let header = $(this);
    let targetId = header.data("target");
    let target = $("#" + targetId);
    header.on("click", function() {
      let isCollapsed = header.hasClass("topics-nav-collapsed");
      if (isCollapsed) {
        header.removeClass("topics-nav-collapsed");
        target.slideDown(150);
      } else {
        header.addClass("topics-nav-collapsed");
        target.slideUp(150);
      }
    });
  });
});

Viewer.onClose = function(viewerInstance) {
  let topicName = viewerInstance.topicName;
  let topicType = viewerInstance.topicType;
  currentTransport.unsubscribe({topicName:topicName});
  $grid.masonry("remove", viewerInstance.card);
  $grid.masonry("layout");
  delete(subscriptions[topicName].viewer);
  delete(subscriptions[topicName]);
  updateStoredSubscriptions();
}

Viewer.onSwitchViewer = (viewerInstance, newViewerType) => {
  let topicName = viewerInstance.topicName;
  let topicType = viewerInstance.topicType;
  if(subscriptions[topicName].viewer !== viewerInstance) console.error("viewerInstance does not match subscribed instance");
  let card = subscriptions[topicName].viewer.card;
  subscriptions[topicName].viewer.destroy();
  delete(subscriptions[topicName].viewer);
  subscriptions[topicName].viewer = new newViewerType(card, topicName, topicType);
};


