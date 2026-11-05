/* Interactive package catalog. Data comes from the JSON block that
   hooks/catalog.py writes into docs/catalog.md. Works with Material's
   instant navigation through the document$ observable. */
(function () {
  "use strict";

  var ROS = { ros2: "ROS 2", ros1: "ROS 1", none: "Non-ROS" };
  var VEH = { usv: "USV", auv: "AUV", rov: "ROV", glider: "Glider", any: "Any" };
  var STATUS = {
    active: "Active", quiet: "Low activity", dormant: "Dormant",
    archived: "Archived", unknown: "Unknown"
  };
  var MONTHS = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"];

  function el(tag, attrs, children) {
    var node = document.createElement(tag);
    Object.keys(attrs || {}).forEach(function (key) {
      if (key === "text") node.textContent = attrs[key];
      else node.setAttribute(key, attrs[key]);
    });
    (children || []).forEach(function (child) { if (child) node.appendChild(child); });
    return node;
  }

  function badge(cls, label) { return el("span", { "class": cls, text: label }); }

  function chipGroup(container, labels, state) {
    Object.keys(labels).forEach(function (key) {
      var chip = el("button", { type: "button", "class": "mrl-chip", "aria-pressed": "false", text: labels[key] });
      chip.addEventListener("click", function () {
        var on = chip.getAttribute("aria-pressed") !== "true";
        chip.setAttribute("aria-pressed", String(on));
        if (on) state.add(key); else state.delete(key);
        container.dispatchEvent(new Event("mrl-change", { bubbles: true }));
      });
      container.appendChild(chip);
    });
  }

  function card(pkg, categories) {
    var meta = el("div", { "class": "mrl-card__meta" });
    pkg.ros.forEach(function (r) { meta.appendChild(badge("mrl-badge mrl-ros-" + r, ROS[r] || r)); });
    pkg.vehicles.forEach(function (v) { meta.appendChild(badge("mrl-badge", VEH[v] || v)); });
    var status = badge("mrl-status mrl-status-" + pkg.status, STATUS[pkg.status]);
    if (pkg.last_commit) {
      var parts = pkg.last_commit.split("-");
      status.textContent += " · " + MONTHS[parseInt(parts[1], 10) - 1] + " " + parts[0];
      status.title = "Last commit " + pkg.last_commit + " on " + (pkg.branch || "default branch");
    }
    meta.appendChild(status);

    var links = el("div", { "class": "mrl-card__links" });
    if (pkg.repo) links.appendChild(el("a", { href: pkg.repo, text: "Source" }));
    if (pkg.url) links.appendChild(el("a", { href: pkg.url, text: "Website / docs" }));

    var desc = pkg.description + (pkg.note ? " " + pkg.note : "");
    return el("article", { "class": "mrl-card" }, [
      el("span", { "class": "mrl-card__cat", text: categories[pkg.category] }),
      el("h3", { text: pkg.name }),
      el("p", { "class": "mrl-card__by", text: pkg.maintainer || "" }),
      el("p", { "class": "mrl-card__desc", text: desc }),
      meta,
      links
    ]);
  }

  function init() {
    var root = document.querySelector("[data-mrl-explorer]");
    var dataNode = document.getElementById("mrl-catalog-data");
    if (!root || !dataNode || root.dataset.ready) return;
    root.dataset.ready = "1";

    var data = JSON.parse(dataNode.textContent);
    var q = root.querySelector("[data-mrl-q]");
    var cat = root.querySelector("[data-mrl-cat]");
    var hideOld = root.querySelector("[data-mrl-active]");
    var grid = root.querySelector("[data-mrl-grid]");
    var count = root.querySelector("[data-mrl-count]");
    var rosSel = new Set();
    var vehSel = new Set();

    Object.keys(data.categories).forEach(function (key) {
      cat.appendChild(el("option", { value: key, text: data.categories[key] }));
    });
    chipGroup(root.querySelector("[data-mrl-ros]"), ROS, rosSel);
    chipGroup(root.querySelector("[data-mrl-veh]"), { usv: "USV", auv: "AUV", rov: "ROV" }, vehSel);

    // Pre-fill from the URL, for links such as catalog/?q=dvl or ?cat=sonar
    var params = new URLSearchParams(window.location.search);
    if (params.get("q")) q.value = params.get("q");
    if (params.get("cat") && data.categories[params.get("cat")]) cat.value = params.get("cat");

    data.packages.forEach(function (pkg) {
      pkg._text = [pkg.name, pkg.description, pkg.maintainer, pkg.note,
        (pkg.tags || []).join(" "), data.categories[pkg.category]].join(" ").toLowerCase();
    });

    function render() {
      var terms = q.value.toLowerCase().split(/\s+/).filter(Boolean);
      var shown = data.packages.filter(function (pkg) {
        if (cat.value && pkg.category !== cat.value) return false;
        if (hideOld.checked && (pkg.status === "dormant" || pkg.status === "archived")) return false;
        if (rosSel.size && !pkg.ros.some(function (r) { return rosSel.has(r); })) return false;
        if (vehSel.size && !pkg.vehicles.some(function (v) { return vehSel.has(v) || v === "any"; })) return false;
        return terms.every(function (t) { return pkg._text.indexOf(t) !== -1; });
      });
      grid.textContent = "";
      shown.forEach(function (pkg) { grid.appendChild(card(pkg, data.categories)); });
      if (!shown.length) {
        grid.appendChild(el("div", { "class": "mrl-empty", text: "No match. Clear a filter, or add the project to data/catalog.yml." }));
      }
      count.textContent = "Showing " + shown.length + " of " + data.packages.length + " projects";
    }

    [q, cat, hideOld].forEach(function (input) { input.addEventListener("input", render); });
    root.addEventListener("mrl-change", render);
    render();
  }

  if (typeof document$ !== "undefined") document$.subscribe(init);
  else document.addEventListener("DOMContentLoaded", init);
})();
