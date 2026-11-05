/* Render math written with pymdownx.arithmatex (generic mode) using KaTeX. */
document$.subscribe(function () {
  if (typeof renderMathInElement === "undefined") return;
  document.querySelectorAll(".arithmatex").forEach(function (node) {
    renderMathInElement(node, {
      delimiters: [
        { left: "\\(", right: "\\)", display: false },
        { left: "\\[", right: "\\]", display: true }
      ],
      throwOnError: false
    });
  });
});
